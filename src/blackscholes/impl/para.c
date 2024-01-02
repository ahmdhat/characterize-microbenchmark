/* para.c
 *
 * Author:
 * Date  :
 *
 *  Description
 */

/* Standard C includes */
#include <stdlib.h>
#include <math.h>

/* Include common headers */
#include "common/macros.h"
#include "common/types.h"

#include <stdio.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <sys/sysctl.h>
#include <pthread.h>

/* Include application-specific headers */
#include "include/types.h"

#define SYSCTL_CORE_COUNT "machdep.cpu.core_count"

#define inv_sqrt_2xPI 0.39894228040143270286

typedef struct cpu_set
{
  uint32_t count;
} cpu_set_t;

static inline void
CPU_ZERO(cpu_set_t *cs) { cs->count = 0; }

static inline void
CPU_SET(int num, cpu_set_t *cs) { cs->count |= (1 << num); }

static inline int
CPU_ISSET(int num, cpu_set_t *cs) { return (cs->count & (1 << num)); }

int sched_getaffinity(pid_t pid, size_t cpu_size, cpu_set_t *cpu_set)
{
  int32_t core_count = 0;
  size_t len = sizeof(core_count);
  int ret = sysctlbyname(SYSCTL_CORE_COUNT, &core_count, &len, 0, 0);
  if (ret)
  {
    printf("error while get core count %d\n", ret);
    return -1;
  }
  cpu_set->count = 0;
  for (int i = 0; i < core_count; i++)
  {
    cpu_set->count |= (1 << i);
  }

  return 0;
}

int pthread_setaffinity_np(pthread_t thread, size_t cpu_size,
                           cpu_set_t *cpu_set)
{
  thread_port_t mach_thread;
  int core = 0;

  for (core = 0; core < 8 * cpu_size; core++)
  {
    if (CPU_ISSET(core, cpu_set))
      break;
  }
  thread_affinity_policy_data_t policy = {core};
  mach_thread = pthread_mach_thread_np(thread);
  thread_policy_set(mach_thread, THREAD_AFFINITY_POLICY,
                    (thread_policy_t)&policy, 1);
  return 0;
}

float CNDF_para(float InputX)
{
  int sign;
  float OutputX;
  float xInput;
  float xNPrimeofX;
  float expValues;
  float xK2;
  float xK2_2, xK2_3;
  float xK2_4, xK2_5;
  float xLocal, xLocal_1;
  float xLocal_2, xLocal_3;
  // Check for negative value of InputX
  if (InputX < 0.0)
  {
    InputX = -InputX;
    sign = 1;
  }
  else
    sign = 0;
  xInput = InputX;
  // Compute NPrimeX term common to both four & six decimal accuracy calcs
  expValues = exp(-0.5f * InputX * InputX);
  xNPrimeofX = expValues;
  xNPrimeofX = xNPrimeofX * inv_sqrt_2xPI;
  xK2 = 0.2316419 * xInput;
  xK2 = 1.0 + xK2;
  xK2 = 1.0 / xK2;
  xK2_2 = xK2 * xK2;
  xK2_3 = xK2_2 * xK2;
  xK2_4 = xK2_3 * xK2;
  xK2_5 = xK2_4 * xK2;
  xLocal_1 = xK2 * 0.319381530;
  xLocal_2 = xK2_2 * (-0.356563782);
  xLocal_3 = xK2_3 * 1.781477937;
  xLocal_2 = xLocal_2 + xLocal_3;
  xLocal_3 = xK2_4 * (-1.821255978);
  xLocal_2 = xLocal_2 + xLocal_3;
  xLocal_3 = xK2_5 * 1.330274429;
  xLocal_2 = xLocal_2 + xLocal_3;
  xLocal_1 = xLocal_2 + xLocal_1;
  xLocal = xLocal_1 * xNPrimeofX;
  xLocal = 1.0 - xLocal;
  OutputX = xLocal;
  if (sign)
  {
    OutputX = 1.0 - OutputX;
  }
  return OutputX;
}

float blackScholesSingle(float sptprice, float strike, float rate, float volatility,
                         float otime, char otype, float timet)
{
  float OptionPrice;
  // local private working variables for the calculation
  float xStockPrice;
  float xStrikePrice;
  float xRiskFreeRate;
  float xVolatility;
  float xTime;
  float xSqrtTime;
  float logValues;
  float xLogTerm;
  float xD1;
  float xD2;
  float xPowerTerm;
  float xDen;
  float d1;
  float d2;
  float FutureValueX;
  float NofXd1;
  float NofXd2;
  float NegNofXd1;
  float NegNofXd2;
  xStockPrice = sptprice;
  xStrikePrice = strike;
  xRiskFreeRate = rate;
  xVolatility = volatility;
  xTime = otime;
  xSqrtTime = sqrt(xTime);
  logValues = log(sptprice / strike);
  xLogTerm = logValues;
  xPowerTerm = xVolatility * xVolatility;
  xPowerTerm = xPowerTerm * 0.5;
  xD1 = xRiskFreeRate + xPowerTerm;
  xD1 = xD1 * xTime;
  xD1 = xD1 + xLogTerm;
  xDen = xVolatility * xSqrtTime;
  xD1 = xD1 / xDen;
  xD2 = xD1 - xDen;
  d1 = xD1;
  d2 = xD2;
  NofXd1 = CNDF_para(d1);
  NofXd2 = CNDF_para(d2);
  FutureValueX = strike * (exp(-(rate) * (otime)));
  if (otype == 'C')
  {
    OptionPrice = (sptprice * NofXd1) - (FutureValueX * NofXd2);
  }
  else
  {
    NegNofXd1 = (1.0 - NofXd1);
    NegNofXd2 = (1.0 - NofXd2);
    OptionPrice = (FutureValueX * NegNofXd2) - (sptprice * NegNofXd1);
  }
  return OptionPrice;
}

void *blackScholesPara(void *args)
{
  args_t *parsed_args = (args_t *)args;
  /* Get all the arguments */
  register size_t size = parsed_args->num_stocks;
  float *sptPrice = parsed_args->sptPrice;
  float *strike = parsed_args->strike;
  float *rate = parsed_args->rate;
  float *volatility = parsed_args->volatility;
  float *otime = parsed_args->otime;
  char *otype = parsed_args->otype;
  float *output = parsed_args->output;

  for (register int i = 0; i < size; i++)
  {
    output[i] = blackScholesSingle(
        sptPrice[i],
        strike[i],
        rate[i],
        volatility[i],
        otime[i],
        otype[i],
        0);
  }

  return NULL;
}

/* Alternative Implementation */
void *impl_parallel(void *args)
{
  args_t *parsed_args = (args_t *)args;

  /* Get all the arguments */
  register size_t size = parsed_args->num_stocks;
  float *sptPrice = parsed_args->sptPrice;
  float *strike = parsed_args->strike;
  float *rate = parsed_args->rate;
  float *volatility = parsed_args->volatility;
  float *otime = parsed_args->otime;
  char *otype = parsed_args->otype;
  float *output = parsed_args->output;

  register size_t nthreads = parsed_args->nthreads;
  register size_t cpu = parsed_args->cpu;

  /* Create all threads */
  pthread_t tid[nthreads];
  args_t targs[nthreads];
  cpu_set_t cpuset[nthreads];

  /* Amount of work per thread */
  size_t size_per_thread = size / nthreads;
  size_t remaining = size % nthreads;

  for (int i = 0; i < nthreads; i++)
  {
    /* Initialize the argument structure */
    targs[i].num_stocks = size_per_thread;
    targs[i].output = output;
    targs[i].sptPrice = sptPrice;
    targs[i].strike = strike;
    targs[i].rate = rate;
    targs[i].volatility = volatility;
    targs[i].otime = otime;
    targs[i].otype = otype;

    output += targs[i].num_stocks;
    sptPrice += targs[i].num_stocks;
    strike += targs[i].num_stocks;
    rate += targs[i].num_stocks;
    volatility += targs[i].num_stocks;
    otime += targs[i].num_stocks;
    otype += targs[i].num_stocks;

    targs[i].cpu = (cpu + i) % nthreads;
    targs[i].nthreads = nthreads;

    /* Affinity */
    CPU_ZERO(&(cpuset[i]));
    CPU_SET(targs[i].cpu, &(cpuset[i]));

    /* Set affinity */
    if (i == 0)
    {
      tid[i] = pthread_self();
    }
    else
    {
      int __attribute__((unused)) res =
          pthread_create(&tid[i], NULL, blackScholesPara, (void *)&targs[i]);
    }

    int __attribute__((unused)) res_affinity = pthread_setaffinity_np(tid[i],
                                                                      sizeof(cpuset[i]), &(cpuset[i]));
  }

  if (nthreads > 0)
  {
    /* Perform one portion of the work */
    for (int i = 0; i < targs[0].num_stocks; i++)
    {
      ((float *)targs[0].output)[i] = blackScholesSingle(
          (targs[0].sptPrice)[i],
          (targs[0].strike)[i],
          (targs[0].rate)[i],
          (targs[0].volatility)[i],
          (targs[0].otime)[i],
          (targs[0].otype)[i],
          0);
    }

    /* Perform trailing elements */
    for (int i = size - remaining; i < size; i++)
    {
      ((float *)targs[0].output)[i] = blackScholesSingle(
          (targs[0].sptPrice)[i],
          (targs[0].strike)[i],
          (targs[0].rate)[i],
          (targs[0].volatility)[i],
          (targs[0].otime)[i],
          (targs[0].otype)[i],
          0);
    }
  }

  /* Wait for all threads to finish execution */
  for (int i = 0; i < nthreads; i++)
  {
    pthread_join(tid[i], NULL);
  }

  return NULL;
}
