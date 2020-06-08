// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.
//
// HillClimbing.h
//

// Defines classes for the ThreadPool's HillClimbing concurrency-optimization
// algorithm.
// https://github.com/dotnet/coreclr/blob/master/src/vm/hillclimbing.h
//

#pragma once

#include <chrono>
#include <complex.h>

#ifdef __linux__
#include <fcntl.h>
#include <zconf.h>
#elif _WIN32
#include <windows.h>
typedef struct _PROCESS_CPU_INFORMATION
{
  LARGE_INTEGER idle_time;
  LARGE_INTEGER kernel_time;
  LARGE_INTEGER user_time;
} PROCESS_CPU_INFORMATION;
#else
#endif

enum hill_climbing_state_transitions
{
  WARMUP,
  INITIALIZING,
  CLIMBING_MOVE,
  STABILIZING
};

typedef std::chrono::milliseconds adj_type;

class hill_climbing
{
private:
  int m_wave_period;
  int m_tasks_to_measure;
  double m_target_throughtput_ratio;
  double m_target_signal_to_noise_ratio;
  double m_max_change_per_second;
  double m_max_change_per_task;
  int m_max_thread_wave_magnitude;
  unsigned long m_task_interval_low;
  double m_thread_magnitude_multiplier;
  unsigned long m_tasks_interval_high;
  double m_throughput_error_smoothing_factor;
  double m_gain_exponent;
  double m_max_task_error;

  double m_current_control_setting;
  long long int m_total_tasks;
  int m_last_thread_count;
  double m_elapsed_since_last_change; //elapsed seconds since last thread count change
  double m_completions_since_last_change; //number of completions since last thread count change

  double m_average_throughput_noise;

  double *m_tasks; //Circular buffer of the last m_tasks_to_measure tasks
  double *m_thread_counts; //Thread counts effective at each of m_tasks

  unsigned int m_current_task_interval;

  int m_accumulated_completion_count;
  double m_accumulated_task_duration;

  void change_thread_count(int new_thread_count,
                           hill_climbing_state_transitions transition);

  bool is_logging;
  int cpu_utilization;

#ifdef __linux__
  int get_cpu_busy();
#elif _WIN32
  PROCESS_CPU_INFORMATION cpu_information;
  LARGE_INTEGER from_filetime(const FILETIME &ft);
  int get_cpu_busy(PROCESS_CPU_INFORMATION *p_old_info);
#else
#endif

public:

  void initialize(bool is_logging_thread_count);

  complex get_wave_component(double* tasks, int tasks_count, double period);

  int update(int current_thread_count, double task_duration,
             int num_completions, adj_type *p_new_adj_interval);

  void force_change(int new_thread_count, hill_climbing_state_transitions transition);

  void log_hill_climbing(int new_thread_count, double throughput, hill_climbing_state_transitions transition);
};

class hill_climbing_mgr
{
public:
  static volatile unsigned long m_thread_pool_completion_count;
  static hill_climbing hill_climbing_instance;

  /** Minimumum number of threads in this pool.*/
  static unsigned int m_min_threads;

  /** Maximimum number of threads in this pool. */
  static unsigned int m_max_threads;

  //static int cpu_utilization;

  static unsigned long PriorCompletedWorkRequests;
  static long long int current_task_start_time;
  static unsigned long current_count_threads;
};
