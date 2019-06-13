#!/usr/bin/env python

###################################################################################################
# Distribution authorized to U.S. Government agencies and their contractors. Other requests for
# this document shall be referred to the MIT Lincoln Laboratory Technology Office.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# (c) 2019 Massachusetts Institute of Technology.

#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

import sys
import time
import logging
import argparse
import numpy as np

from tesse.env import Env
from tesse.msgs import DataRequest


def main():
    parser = argparse.ArgumentParser('timing')
    parser.add_argument('--simulation-ip', type=str, default='localhost',
                        help='The ip address for the simulation')
    parser.add_argument('--own-ip', type=str, default='localhost',
                        help='The ip address for this process')
    parser.add_argument('--position-port', type=int, default=9000,
                        help='The port number for the position interface')
    parser.add_argument('--metadata-port', type=int, default=9001,
                        help='The port number for the metadata interface')
    parser.add_argument('--image-port', type=int, default=9002,
                        help='The port number for the image interface')
    parser.add_argument('-n', '--num-trials', type=int, default=100,
                        help='The number of trials to use to measure timing')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Print debug information')
    args = parser.parse_args()

    logging.basicConfig(
        format='%(relativeCreated)6d [%(levelname)s] %(message)s',
        level=logging.DEBUG if args.verbose else logging.INFO,
    )

    env = Env(
        simulation_ip=args.simulation_ip,
        own_ip=args.own_ip,
        position_port=args.position_port,
        metadata_port=args.metadata_port,
        image_port=args.image_port,
    )

    times = []
    for i in range(args.num_trials):
        start = time.time()
        response = env.request(DataRequest())
        if response is None:
            logging.warning("Environment failed to respond on trial {}".format(i))
            continue
        times.append(time.time() - start)
        logging.debug("Trial {:3d}/{}: {:0.3f}".format(i, args.num_trials, times[-1]))
    if len(times) > 0:
        logging.info("Number of trials:   {}".format(len(times)))
        logging.info("Average time:       {:0.3f} ms".format(100 * np.mean(times)))
        logging.info("Standard deviation: {:0.3f} ms".format(100 * np.std(times)))
        logging.info("Frames per second:  {:0.3f} Hz".format(1 / np.mean(times)))
    else:
        logging.warning('Could not measure timing!')


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(1)
