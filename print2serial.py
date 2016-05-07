# Authors:       Sergiy Kolodyazhnyy, Anthony Thao,
# Course :       Interface Techniques , EET 4340 , Sprint 2016
# Assignment:    Final project
# Description:   The python code bellow prints a selections menu
#                for the user, and sends those choices from Raspberry
#                Pi to PIC18F87J11 Microcontroller
# Tools used: Ubuntu Snappy 15.04, vim, Python version 2.7

from time import sleep
import sys
import subprocess
import os
import sys

choices = [ ('a','Reprint char on LCD'), \
            ('b','Read potentiometer value') , \
            ('c','Set LEDs') \
          ]
#---------------------
# Function definitions
#---------------------

def print_choices():
  print "Select action:"
  for choice in choices :
     print choice[0],')',choice[1]

def write_to_serial(input_char):
   if os.path.exists('/dev/ttyACM0'):
     fd = open('/dev/ttyACM0','w')
     fd.write(input_char)
     fd.close()
   else:
     print >> sys.stderr, "ERR: serial device disconnected"
     raw_input()

def clear_screen():
  return_code = subprocess.call("clear")

# FIXME:
# The same function , read_serial(), has been tested by reading
# raspberry output on Ubuntu terminal,and the function
# performs correctly, however the board still doesn't
# transmit anything. Likely it is hardware or hardware
# configuration issue

def read_serial():
  with open("/dev/ttyACM0", "r") as serial:
         print "Received from MCU: %s" % serial.read(1)
         raw_input()
         serial.close()
#-----------------
# MAIN
#-----------------
while True:
  clear_screen()
  print_choices()
  char = raw_input()
  write_to_serial(char)
  if char == 'a' or char == 'c':
     print "Which char to send?"
     char2 = raw_input()
     write_to_serial(char2)
     #if char == 'a':
     #   read_serial()
