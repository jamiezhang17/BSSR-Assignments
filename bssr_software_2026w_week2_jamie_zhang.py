# Name: Jamie Zhang
# Year and program: 2T9 + PEY ECE
# UofT email: jamiejamie.zhang@mail.utoronto.ca

from dataclasses import dataclass
from logging import getLogger
from time import sleep, time
from typing import ClassVar

from revolution.application import Application
from revolution.environment import Endpoint
from revolution.utilities import Direction
from revolution.worker import Worker

_logger = getLogger(__name__)


@dataclass
class Motor(Application):
    endpoint: ClassVar[Endpoint] = Endpoint.MOTOR

    def _setup(self) -> None:
        super()._setup()

        self._control_worker = Worker(target=self._control)

        self._control_worker.start()

    def _teardown(self) -> None:
        self._control_worker.join()

    def _control(self) -> None:

        while ( # run while true
            # not False = True 
                not self._stoppage.wait( # if NO "stop" signal was recieved  
                                            # (meaning the timeout was reached before the motor sends a "stop" signal)
                                            # returns FAlse
                    # motor timeout = a pre-set time limit 
                    self.environment.settings.motor_control_timeout, # contiue waiting 
                    # if "stop" signal is recieved, .wait = True and not True = False so loop breaks
                )
        ):
            with self.environment.contexts() as contexts: # store self.environment.contexts() resource as a variable "contexts"
                # get states from contexts
                status_input = contexts.motor_status_input # current status of the motor (extracted from environment)
                # ADDED ########## 
                num_error_flags = contexts.motor_controller_error_flags # get the error flags from motor environment to detect any errors
                MOTOR_MAX_CURRENT = contexts.motor_bus_current_limit # # store the current limits as a constant from environment settings
                current_flag = contexts.motor_controller_limit_flags # get the current flags from the battery 
                current_input = contexts.motor_controller_bus_current # get the current from environments
                


            # status_input here is basically a bool representing the state of
            # the battery relay
            if status_input: # if status is true (battery on)
                if not previous_status_input: # run if status is not the previous status (eg. off to on) 
                    self.environment.peripheries.motor_wavesculptor22.reset() # only reset if off to on which prevents resetting continuously while on
                else:
                    # motor control logic
                    pass # do nothing (normal operation)
            else: # if status is false (battery off)
                with self.environment.contexts() as contexts:
                    contexts.motor_cruise_control_status_input = False # diable the cruise control for safety

            # TODO: Implement motor controller automatic restart here
            # You need to add an if statement to check for error
            # You can add things outside the while loop but if you do so 
            # please comment there to make it easier to see the changes
            
            # check for errors using error flags from environment 
            # (mentioned in Status Information from document)
            if num_error_flags != 0:
                should_reset = True # trigger for resetting the motor 

            # check for over current errors 
            if current_flag != 0 or current_input > MOTOR_MAX_CURRENT: # reset if flagged as error or the current is greater than the max limit
                should_reset = True 
                
            # if detected errors, reset 
            if should_reset:
                
                # call WaveSculptor22 to reset (just like earlier)
                self.environment.peripheries.motor_wavesculptor22.reset()
    
                # reset contexts after 
                with self.environment.contexts() as contexts:
                    contexts.motor_controller_error_flags = 0 # reset error flags to 0 
                    contexts.motor_controller_limit_flags = 0
                    contexts.motor_cruise_control_status_input = False # condition to run the while loop again (restarting)
                    
                
            # -------------------------------------------


            '''
            # command to restart motor controller
            self.environment.peripheries.motor_wavesculptor22.reset()

            # example on how to access internal states
            # you must acquire the lock by the with statement
            with self.environment.contexts() as contexts:
                # read
                velocity = contexts.motor_velocity
                # write
                contexts.motor_velocity = 0
            '''

            previous_status_input = status_input 

