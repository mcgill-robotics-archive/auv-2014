#! /usr/bin/env python


import sixaxis

sixaxis.init("/dev/input/js0")

while(True):
    state = sixaxis.get_state()
    if(state['triangle'] == True):
	print("_______________TRIANGLE PRESSED _____________")
    if(state['square'] == True):
	print("++++++++++++++SQUARE++++++++++++++++")
	        

    
