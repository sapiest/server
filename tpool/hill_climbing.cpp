// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.
//
// HillClimbing.cpp
//

// Defines classes for the ThreadPool's HillClimbing concurrency-optimization
// algorithm.
// https://github.com/dotnet/coreclr/blob/master/src/vm/hillclimbing.cpp
//

#include <random>
#include <complex.h>
#include <iostream>

#include "hill_climbing.h"

const double pi = 3.141592653589793;

void hill_climbing::initialize(bool is_logging_thread_count)
{
  m_wave_period = 4;
  m_max_thread_wave_magnitude = 20;
  m_thread_magnitude_multiplier = 100 / 100.0;
  m_tasks_to_measure = m_wave_period * 8;
  //"The 'cost' of a thread.  0 means drive for increased throughput regardless
  // of thread count; higher values bias more against higher thread counts.");
  // bias - The 'cost' of a thread. 0 means drive for increased throughput
  // regardless of thread count; higher values bias more against higher thread
  // counts.
  m_target_throughtput_ratio = 15 / 100.0;
  m_target_signal_to_noise_ratio = 300 / 100.0;
  m_max_change_per_second = 4;
  m_max_change_per_task = 20;
  m_task_interval_low = 10;
  m_tasks_interval_high = 200;
  m_throughput_error_smoothing_factor = 1 / 100.0;
  m_gain_exponent = 200 / 100.0;
  m_max_task_error = 15 / 100.0;

  m_current_control_setting = 0;
  m_total_tasks = 0;
  m_last_thread_count = 0;
  m_average_throughput_noise = 0;
  m_elapsed_since_last_change = 0;
  m_completions_since_last_change = 0;
  m_accumulated_completion_count = 0;
  m_accumulated_task_duration = 0;

  m_tasks = new double[m_tasks_to_measure];
  m_thread_counts = new double[m_tasks_to_measure];
  is_logging = is_logging_thread_count;

  cpu_utilization = 0;
#ifdef __linux__
#elif _WIN32
  cpu_information = {{0}, {0}, {0}};
  get_cpu_busy(&cpu_information);
#else
#endif
}

void hill_climbing::change_thread_count(
    int new_thread_count, hill_climbing_state_transitions transition)
{
  m_last_thread_count = new_thread_count;
//  std::uniform_int_distribution<int> uid(m_task_interval_low, m_tasks_interval_high);
//  std::random_device rd;
//  m_current_task_interval = uid(rd);
  srand( time(nullptr) );
  m_current_task_interval = m_task_interval_low + rand() % m_tasks_interval_high +1;
  double throughput =(m_elapsed_since_last_change > 0)? (m_completions_since_last_change / m_elapsed_since_last_change): 0;
  /** logging of thread count */
  log_hill_climbing(new_thread_count, throughput, transition);
  m_elapsed_since_last_change = 0;
  m_completions_since_last_change = 0;
}

/** Counting new concurrency*/
int hill_climbing::update(int current_thread_count, double task_duration,
                          int num_completions, adj_type *p_new_adj_interval)
{
  /** if thread_count changed from different thread or method*/
  if (m_last_thread_count != current_thread_count)
  {
    force_change(current_thread_count, INITIALIZING);
  }

#ifdef __linux__
  cpu_utilization = get_cpu_busy();
#elif _WIN32
  cpu_utilization = get_cpu_busy(&cpu_information);
#else
#endif

  m_elapsed_since_last_change += task_duration;
  m_completions_since_last_change += num_completions;

  task_duration += m_accumulated_task_duration;
  num_completions += m_accumulated_completion_count;
  double throughput = (double) num_completions / task_duration;

  //
  // We need to make sure we're collecting reasonably accurate data.  Since
  // we're just counting the end of each work item, we are goinng to be missing
  // some data about what really happened during the sample interval.  The
  // count produced by each thread includes an initial work item that may have
  // started well before the start of the interval, and each thread may have
  // been running some new work item for some time before the end of the
  // interval, which did not yet get counted.  So our count is going to be off
  // by +/- threadCount workitems.
  //
  // The exception is that the thread that reported to us last time definitely
  // wasn't running any work at that time, and the thread that's reporting now
  // definitely isn't running a work item now.  So we really only need to
  // consider threadCount-1 threads.
  //
  // Thus the percent error in our count is +/- (threadCount-1)/numCompletions.
  //
  // We cannot rely on the frequency-domain analysis we'll be doing later to
  // filter out this error, because of the way it accumulates over time.  If
  // this sample is off by, say, 33% in the negative direction, then the next
  // one likely will be too.  The one after that will include the sum of the
  // completions we missed in the previous samples, and so will be 33%
  // positive.  So every three samples we'll have two "low" samples and one
  // "high" sample.  This will appear as periodic variation right in the
  // frequency range we're targeting, which will not be filtered by the
  // frequency-domain translation.
  //

  if ((m_total_tasks > 0 &&
       ((current_thread_count - 1.0) / num_completions) >= m_max_task_error) ||
      throughput == 0)
  {
    // not accurate enough yet.  Let's accumulate the data so far, and tell the
    // ThreadPool to collect a little more.
    m_accumulated_task_duration = task_duration;
    m_accumulated_completion_count = num_completions;
    *p_new_adj_interval = adj_type(10);
    return current_thread_count;
  }

  //
  // We've got enough data for our sample; reset our accumulators for next
  // time.
  //
  m_accumulated_completion_count = 0;
  m_accumulated_task_duration = 0;

  //
  // Add the current thread count and throughput sample to our history
  //
  int task_index = m_total_tasks % m_tasks_to_measure;
  m_tasks[task_index] = throughput;
  m_thread_counts[task_index] = current_thread_count;
  m_total_tasks++;

  //
  // Set up defaults for our metrics
  //
  complex thread_wave_component = 0;
  complex throughput_wave_component = 0;
  double throughput_error_estimate = 0;
  complex ratio = 0;
  double confidence = 0;

  hill_climbing_state_transitions transition = WARMUP;

  //
  // How many samples will we use?  It must be at least the three wave periods
  // we're looking for, and it must also be a whole multiple of the primary
  // wave's period; otherwise the frequency we're looking for will fall between
  // two  frequency bands in the Fourier analysis, and we won't be able to
  // measure it accurately.
  //
  int task_count =
      static_cast<int>(fmin(m_total_tasks - 1, m_tasks_to_measure) /
                       m_wave_period) *
      m_wave_period;

  if (task_count > m_wave_period)
  {
    //
    // Average the throughput and thread count samples, so we can scale the
    // wave magnitudes later.
    //
    double task_sum = 0;
    double thread_sum = 0;

    for (int i = 0; i < task_count; i++)
    {
      task_sum += m_tasks[(m_total_tasks - task_count + i) % m_tasks_to_measure];
      thread_sum += m_thread_counts[(m_total_tasks - task_count + i) % m_tasks_to_measure];
    }

    double average_throughtput = task_sum / task_count;
    double average_thread_count = thread_sum / task_count;

    if (average_throughtput > 0 && average_thread_count > 0)
    {

      //
      // Calculate the periods of the adjacent frequency bands we'll be using
      // to measure noise levels. We want the two adjacent Fourier frequency
      // bands.
      //
      double adjacent_period1 =
          task_count / (((double) task_count / (double) m_wave_period) + 1);
      double adjacent_period2 =
          task_count / (((double) task_count / m_wave_period) - 1);

      //
      // Get the the three different frequency components of the throughput
      // (scaled by average throughput).  Our "error" estimate (the amount of
      // noise that might be present in the frequency band we're really
      // interested in) is the average of the adjacent bands.
      //
      throughput_wave_component  =
          get_wave_component(m_tasks, task_count, m_wave_period) /
          average_throughtput;
      throughput_error_estimate =
          abs(get_wave_component(m_tasks, task_count, adjacent_period1)) /
          average_thread_count;
      if (adjacent_period2 <= task_count)
      {
        throughput_error_estimate = fmax(
            throughput_error_estimate,
            abs(get_wave_component(m_tasks, task_count, adjacent_period2)) /
                average_thread_count);
      }

      //
      // Do the same for the thread counts, so we have something to compare to.
      // We don't measure thread count noise, because there is none; these are
      // exact measurements.
      //
      thread_wave_component =
          get_wave_component(m_thread_counts, task_count, m_wave_period) /
          average_thread_count;

      if (m_average_throughput_noise == 0)
      {
        m_average_throughput_noise = throughput_error_estimate;
      }
      else
      {
        m_average_throughput_noise =
            (m_throughput_error_smoothing_factor * throughput_error_estimate) +
            ((1.0 - m_throughput_error_smoothing_factor) *
             m_average_throughput_noise);
      }

      if (abs(thread_wave_component) > 0)
      {
        //
        // Adjust the throughput wave so it's centered around the target wave,
        // and then calculate the adjusted throughput/thread ratio.
        //
        ratio = (throughput_wave_component -
                (m_target_throughtput_ratio * thread_wave_component)) /thread_wave_component;
        transition = CLIMBING_MOVE;
      }
      else
      {
        ratio = 0;
        transition = STABILIZING;
      }

      //
      // Calculate how confident we are in the ratio.  More noise  = = less
      // confident.  This has the effect of slowing down movements that might
      // be affected by random noise.
      //
      double noise_for_confidence =
          fmax(m_average_throughput_noise, throughput_error_estimate);
      if (noise_for_confidence > 0)
      {
        confidence = (abs(thread_wave_component) / noise_for_confidence) /
                    m_target_throughtput_ratio;
      }
      else
      {
        // no noise
        confidence = 1.0;
      }
    }
  }

  //
  // We use just the real part of the complex ratio we just calculated.  If the
  // throughput signal is exactly in phase with the thread signal, this will be
  // the same as taking the magnitude of the complex move and moving that far
  // up.  If they're 180 degrees out of phase, we'll move backward (because
  // this indicates that our changes are having the opposite of the intended
  // effect). If they're 90 degrees out of phase, we won't move at all, because
  // we can't tell wether we're having a negative or positive effect on
  // throughput.
  //
  double move = fmin(1.0, fmax(-1.0, ratio.r));

  //
  // Apply our confidence multiplier.
  //
  move *= fmin(1.0, fmax(0.0, confidence));

  //
  // Now apply non-linear gain, such that values around zero are attenuated,
  // while higher values are enhanced.  This allows us to move quickly if we're
  // far away from the target, but more slowly if we're getting close, giving
  // us rapid ramp-up without wild oscillations around the target.
  //
  double gain = m_max_change_per_second * task_duration;
  move = pow(fabs(move), m_gain_exponent) * (move >= 0.0 ? 1 : -1) * gain;
  move = fmin(move, m_max_change_per_task);

  //
  // If the result was positive, and CPU is > 95%, refuse the move.
  //
  //  if (move > 0.0 && hill_climbing_mgr::cpu_utilization > 95) // check cpu
  //  utilization > 95%
  //  {
  //    move = 0.0;
  //  }

  //
  // Apply the move to our control setting
  //
  m_current_control_setting += move;

  //
  // Calculate the new thread wave magnitude, which is based on the moving
  // average we've been keeping of the throughput error.  This average starts
  // at zero, so we'll start with a nice safe little wave at first.
  //
  int new_thread_wave_magnitude = static_cast<int>(
      0.5 +
      (m_current_control_setting * m_average_throughput_noise *
       m_target_signal_to_noise_ratio * m_thread_magnitude_multiplier * 2.0));
  new_thread_wave_magnitude =
      fmin(new_thread_wave_magnitude, m_max_thread_wave_magnitude);
  new_thread_wave_magnitude = fmax(new_thread_wave_magnitude, 1);

  //
  // Make sure our control setting is within the ThreadPool's limits
  //
  m_current_control_setting =
      fmin(hill_climbing_mgr::m_max_threads - new_thread_wave_magnitude,
           m_current_control_setting);
  m_current_control_setting =
      fmax(hill_climbing_mgr::m_min_threads, m_current_control_setting);

  //
  // Calculate the new thread count (control setting + square wave)
  //
  int new_thread_count = static_cast<int>(
      m_current_control_setting +
      new_thread_wave_magnitude * ((m_total_tasks / (m_wave_period / 2)) % 2));

  //
  // Make sure the new thread count doesn't exceed the ThreadPool's limits
  //
  new_thread_count = fmin(hill_climbing_mgr::m_max_threads, new_thread_count);
  new_thread_count = fmax(hill_climbing_mgr::m_min_threads, new_thread_count);

  //
  // If all of this caused an actual change in thread count, log that as well.
  //
  if (new_thread_count != current_thread_count)
  {
    change_thread_count(new_thread_count, transition);
  }

  // Return the new thread count and sample interval.  This is randomized to
  // prevent correlations with other periodic changes in throughput.  Among
  // other things, this prevents us from getting confused by Hill Climbing
  // instances running in other processes.
  //
  // If we're at minThreads, and we seem to be hurting performance by going
  // higher, we can't go any lower to fix this.  So we'll simply stay at
  // minThreads much longer, and only occasionally try a higher value.
  //

  if (ratio.r < 0.0 && new_thread_count == hill_climbing_mgr::m_min_threads)
  {
    *p_new_adj_interval = adj_type(static_cast<int>(0.5 + m_current_task_interval * (10.0 * fmax(-ratio.r, 1.0))));
  }
  else
  {
    *p_new_adj_interval = adj_type(m_current_task_interval);
  }

  return new_thread_count;
}

/** manually changing thread_count*/
void hill_climbing::force_change(int new_thread_count,
                                 hill_climbing_state_transitions transition)
{
  if (new_thread_count != m_last_thread_count)
  {
    m_current_control_setting += (new_thread_count - m_last_thread_count);
    change_thread_count(new_thread_count, transition);
  }
}

/** log current threads*/
void hill_climbing::log_hill_climbing(
    int new_thread_count, double throughput,
    hill_climbing_state_transitions transition)
{
  if (is_logging)
  {
    fprintf(stderr,
            "Changing concurrency: state %i,"
            "current number of threads in pool %d,"
            "throughput %f\n",
            transition, new_thread_count, throughput);
  }
}

complex hill_climbing::get_wave_component(double *tasks, int tasks_count,
                                          double period)
{

  //
  // Calculate the sinusoid with the given period.
  // We're using the Goertzel algorithm for this.  See
  // http://en.wikipedia.org/wiki/Goertzel_algorithm.
  //
  double w = 2.0 * pi / period;
  double cosine = cos(w);
  double sine = sin(w);
  double coeff = 2.0 * cosine;
  double q0 = 0, q1 = 0, q2 = 0;

  for (int i = 0; i < tasks_count; i++)
  {
    double task = tasks[(m_total_tasks - tasks_count + i) % m_tasks_to_measure];

    q0 = coeff * q1 - q2 + task;
    q2 = q1;
    q1 = q0;
  }

  return complex(q1 - q2 * cosine, q2 * sine) / (double) tasks_count;
}

#ifdef __linux__
int hill_climbing::get_cpu_busy(){
  int file_handler;
  char file_buffer[1024];
  float cpu;

  if((file_handler  = open("/proc/loadavg", O_RDONLY)) < 0){
    return EXIT_FAILURE;
  };

  read(file_handler, file_buffer, sizeof(file_buffer) - 1);
  sscanf(file_buffer, "%f", &cpu);
  close(file_handler);

  //fprintf(stderr, "cpu: %f\n", cpu * 100);

  return static_cast<int>(cpu * 100);
}
#elif _WIN32
LARGE_INTEGER hill_climbing::from_filetime(const FILETIME &ft)
{
  LARGE_INTEGER uli = {0};
  uli.u.LowPart = ft.dwLowDateTime;
  uli.u.HighPart = ft.dwHighDateTime;
  return uli;
}

int hill_climbing::get_cpu_busy(PROCESS_CPU_INFORMATION *p_old_info)
{
  PROCESS_CPU_INFORMATION new_usage;
  new_usage.idle_time = {0};
  new_usage.kernel_time = {0};
  new_usage.user_time = {0};

  FILETIME new_idle, new_kernel, new_user;
  GetSystemTimes(&new_idle, &new_kernel, &new_user);
  new_usage.idle_time = from_filetime(new_idle);
  new_usage.kernel_time = from_filetime(new_kernel);
  new_usage.user_time = from_filetime(new_user);

  uint64_t idl = new_usage.idle_time.QuadPart - p_old_info->idle_time.QuadPart;
  uint64_t ker =
      new_usage.kernel_time.QuadPart - p_old_info->kernel_time.QuadPart;
  uint64_t usr = new_usage.user_time.QuadPart - p_old_info->user_time.QuadPart;

  auto denominator  = (ker + usr + idl);
  if(denominator == 0){
    return 0;
  }

  uint64_t cpu = (ker + usr) * 100 / denominator;

  p_old_info->user_time = new_usage.user_time;
  p_old_info->kernel_time = new_usage.kernel_time;
  p_old_info->idle_time = new_usage.idle_time;

  //fprintf(stderr, "cpu: %llu\n", cpu);

  return static_cast<int>(cpu);
}

#else
#endif

unsigned int hill_climbing_mgr::m_min_threads = 0;
unsigned int hill_climbing_mgr::m_max_threads = 0;
unsigned long hill_climbing_mgr::current_count_threads = 0;
volatile unsigned long hill_climbing_mgr::m_thread_pool_completion_count = 0;
unsigned long hill_climbing_mgr::PriorCompletedWorkRequests = 0;
long long int hill_climbing_mgr::current_task_start_time  =0;
hill_climbing hill_climbing_mgr::hill_climbing_instance = hill_climbing();
