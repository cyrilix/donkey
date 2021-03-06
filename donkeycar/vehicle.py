#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 25 10:44:24 2017

@author: wroscoe
"""
import logging
import time
from abc import ABC, abstractmethod
from threading import Thread
from typing import Any, Dict

from donkeycar.parts.part import Part, ThreadedPart
from .memory import Memory

logger = logging.getLogger(__name__)


class MetricsPublisher(ABC):

    @abstractmethod
    def publish(self, values: Dict[str, Any]):
        pass

    @abstractmethod
    def shutdown(self):
        pass


class Vehicle:
    def __init__(self, mem=None, metrics_publisher: MetricsPublisher = None):

        if not mem:
            mem = Memory()
        self.mem = mem
        self.parts = []
        self.on = True
        self.threads = []
        self.metrics_publisher = metrics_publisher
        self.sleep_time = 0.0

    def register(self, part: Part, run_condition=None):
        self.add(part=part,
                 inputs=part.get_inputs_keys(),
                 outputs=part.get_outputs_keys(),
                 threaded=isinstance(part, ThreadedPart),
                 run_condition=run_condition)

    def add(self, part, inputs=None, outputs=None,
            threaded=False, run_condition=None):
        """
        Method to add a part to the vehicle drive loop.

        Parameters
        ----------
            :param part:
            :param inputs : list
                Channel names to get from memory.
            :param outputs : list
                Channel names to save to memory.
            :param threaded : boolean
                If a part should be run in a separate thread.
            :param run_condition:
        """

        if outputs is None:
            outputs = []
        if inputs is None:
            inputs = []
        p = part
        logger.info('Adding part %s with inputs: %s and outputs: %s.', p.__class__.__name__, inputs, outputs)
        entry = {'part': p,
                 'inputs': inputs,
                 'outputs': outputs,
                 'run_condition': run_condition}

        if threaded:
            t = Thread(target=part.update, args=())
            t.daemon = True
            entry['thread'] = t

        self.parts.append(entry)

    def start(self, rate_hz=10, max_loop_count=None):
        """
        Start vehicle's main drive loop.

        This is the main thread of the vehicle. It starts all the new
        threads for the threaded parts then starts an infinit loop
        that runs each part and updates the memory.

        Parameters
        ----------

        rate_hz : int
            The max frequency that the drive loop should run. The actual
            frequency may be less than this if there are many blocking parts.
        max_loop_count : int
            Maxiumum number of loops the drive loop should execute. This is
            used for testing the all the parts of the vehicle work.
        """

        try:

            self.on = True

            for entry in self.parts:
                if entry.get('thread'):
                    # start the update thread
                    entry.get('thread').start()

            # wait until the parts warm up.
            logger.info('Starting vehicle...')
            time.sleep(1)

            loop_count = 0
            while self.on:
                start_time = time.time()
                loop_count += 1

                self.update_parts()
                self._publish_metrics(rate_hz)

                # stop drive loop if loop_count exceeds max_loopcount
                if max_loop_count and loop_count > max_loop_count:
                    self.on = False

                sleep_time = 1.0 / rate_hz - (time.time() - start_time)
                self.sleep_time = sleep_time
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def update_parts(self):
        """
        loop over all parts
        """
        for entry in self.parts:
            # don't run if there is a run condition that is False
            run = True
            if entry.get('run_condition'):
                run_condition = entry.get('run_condition')
                run = self.mem.get([run_condition])[0]
                # print('run_condition', entry['part'], entry.get('run_condition'), run)

            if run:
                p = entry['part']
                # get inputs from memory
                inputs = self.mem.get(entry['inputs'])

                # run the part
                if entry.get('thread'):
                    outputs = p.run_threaded(*inputs)
                else:
                    outputs = p.run(*inputs)

                # save the output to memory
                if outputs is not None:
                    self.mem.put(entry['outputs'], outputs)

    def stop(self):
        logger.info('Shutting down vehicle and its parts...')
        for entry in self.parts:
            try:
                entry['part'].shutdown()
            except Exception as e:
                logging.exception(e)
        if self.metrics_publisher:
            self.metrics_publisher.shutdown()
        logger.debug(self.mem.d)

    def _publish_metrics(self, rate_htz):
        if self.metrics_publisher:
            metrics = dict([x for x in self.mem.d.items() if isinstance(x[0], str) and not x[0].startswith('_')])
            metrics['sleep_time'] = self.sleep_time
            metrics['rate_htz'] = rate_htz
            self.metrics_publisher.publish(metrics)
