/*
 *  Montefiore Robocup
 *  ==================
 *
 *  main.c: Main code for the motherboard.
 */

#include <stdio.h>
#include <string.h>

#include "./utils/communication.h"
#include "./utils/usb.h"
#include "./world/world.h"
#include "./communication/task.h"
#include "./expert/expert-collisions.h"
#include "./log/log.h"
#include "./reflex/hold_reflex.h"

/***  Private global variable(s)  ***/

static char *dev_name = NULL;

/***  Prototypes of private functions  ***/

static void usage(const char *);
static int  parse_args(int, char *[]);

/***  Private functions  ***/

/* Displays man page */

static void  usage(const char *pgm_name)
{
  fprintf(stderr, "%s: Main code of the motherboard.\n"
                  "Usage: %s [<options>]\n"
                  "Option(s):\n"
                  "    -d <dev>\n"
                  "        Use specific USB device\n",
          pgm_name, pgm_name);
}

/*
 * Parses command line arguments and sets global variables
 * accordingly.  Returns -1 in case of erroneous usage, and 0
 * otherwise.
 */

static int  parse_args(int argc, char *argv[])
{
  unsigned  i;

  for (i = 1; i < argc; i++)
    {
      if (!strcmp(argv[i], "-d") || dev_name)
        {
          if (++i >= argc)
            return -1;

          dev_name = argv[i];
        }

      if (argv[i][0] == '-')
        return -1;
    }

  return 0;
}

/***  Entry point  ***/

int  main(int argc, char *argv[])
{
  FILE *f;

  /* Initialize world */

  init_world();

  /* Initialize log */

  if (log_init(LOG_DEBUG) != 0) {
      log_critical("Error during journal initialization\n");
  }

  log_normal("PID: %d\n", getpid());

  if (parse_args(argc, argv))
    {
      usage(argv[0]);
      return -1;
    }

  f = usb_connect(dev_name);
  if (!f)
    return -1;

  if (com_boot_dbs(f))
    return -1;


  pthread_t thread_expert_collis;

  if(pthread_create(&thread_expert_collis, NULL, expert_collisions_main, NULL)) {
    printf("Error while creating a thread\n");
    log_critical("Error while creating the collision thread \n");
    return -1;
  }

  pthread_t thread_hold_reflex;

  if(pthread_create(&thread_hold_reflex, NULL, hold_reflex_main, NULL)) {
    printf("Error while creating a thread\n");
    log_critical("Error while creating a the hold reflex thread \n");
    return -1;
  }



  for (;;)
    {
      if (com_uplink(f)) {
        printf("Error in uplink\n");
        log_critical("Error in com uplink\n");
        continue;
      }

      com_downlink(f);

    }
}
