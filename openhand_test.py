#!/usr/bin/python

import openhand_conf
import pypot.robot
import pypot.primitive
import numpy as np
import time

class grabPrimitive(pypot.primitive.Primitive):
    def run(self):
        self.robot.base_motor.goal_position = self.robot.base_motor.upper_limit
        self.robot.f1.goal_position = self.robot.f1.upper_limit
        self.robot.f2.goal_position = self.robot.f2.upper_limit
        self.robot.thumb.goal_position = self.robot.thumb.upper_limit
        time.sleep(1)

class ungrabPrimitive(pypot.primitive.Primitive):
    def run(self):
        self.robot.base_motor.goal_position = self.robot.base_motor.upper_limit
        self.robot.f1.goal_position = self.robot.f1.lower_limit
        self.robot.f2.goal_position = self.robot.f2.lower_limit
        self.robot.thumb.goal_position = self.robot.thumb.lower_limit
        time.sleep(1)

class grabDemoPrimitive(pypot.primitive.Primitive):
    def run(self, duration=4.0):
        f1 = self.robot.f1
        f2 = self.robot.f2
        thumb = self.robot.thumb
        base = self.robot.base_motor
        
        while self.elapsed_time < duration:
            base.goal_position = base.upper_limit - 35
            thumb.goal_position = thumb.lower_limit + self.elapsed_time*(thumb.upper_limit - thumb.lower_limit)/duration
            f1.goal_position = f1.lower_limit + self.elapsed_time*(f1.upper_limit - f1.lower_limit)/duration
            f2.goal_position = f2.lower_limit + self.elapsed_time*(f2.upper_limit - f2.lower_limit)/duration
            print f1.present_load, f2.present_load, thumb.present_load
            time.sleep(0.02)

        while self.elapsed_time < 2*duration:
            base.goal_position = base.upper_limit
            if(thumb.present_load < 10):
                thumb.goal_position = thumb.upper_limit + -1*(self.elapsed_time - duration)*(thumb.upper_limit - thumb.lower_limit)/duration
            if(f1.present_load < 10):
                f1.goal_position = f1.upper_limit + -1*(self.elapsed_time - duration)*(f1.upper_limit - f1.lower_limit)/duration
            if(f2.present_load < 10):
                f2.goal_position = f2.upper_limit + -1*(self.elapsed_time - duration)*(f2.upper_limit - f2.lower_limit)/duration
            print f1.present_load, f2.present_load, thumb.present_load
            time.sleep(0.02)

hand = pypot.robot.from_config(openhand_conf.openhand_config)
grab = grabPrimitive(hand)
ungrab = ungrabPrimitive(hand)
grabDemo = grabDemoPrimitive(hand)

if __name__ == '__main__':
    grab.start()
    time.sleep(1)
    ungrab.start()
