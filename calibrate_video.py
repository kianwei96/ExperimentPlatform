import eyelink
from psychopy import visual, core
import rospy
import pickle

def main(edffile='test.edf',
         screen_width=800,
         screen_height=600,
         full_screen=False,
         dot_duration=1.0):
    # read set up
    # with open('eparams.pkl','rb') as handle:
	#     exp_info = pickle.load(handle)
    rospy.init_node("eyelink_sync")
    # create a window
    win = visual.Window(
        size=(screen_width, screen_height), fullscr=full_screen, screen=0,
        allowGUI=True, allowStencil=False,
        monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
        blendMode='avg', useFBO=False)

    # use pixels as units
    win.setUnits('pix')

    # create a visual stimulus for the dots
    dot = visual.Circle(win, radius=30.0,
                        fillColor="white",
                        lineColor="white", 
                        units='pix',
                        fillColorSpace='rgb',
                        lineColorSpace='rgb')
    margins = [0.8*win.size[0]//2, 0.8*win.size[1]//2]
    dot_pos =   [   
                    (-margins[0], -margins[1]), (0, -margins[1]), (margins[0], -margins[1]),
                    (-margins[0],           0), (0,           0), (margins[0],           0),
                    (-margins[0],  margins[1]), (0,  margins[1]), (margins[0],  margins[1])
                ]
    
    # keep track of time
    clock = core.Clock()

    # establish a connection to the tracker
    tracker = eyelink.Eyelink(win, edffile)

    # start the recording
    tracker.start_recording()
   
    idx = 0
    clock.reset()
    win.callOnFlip(lambda: tracker.send_message('dot pos: {}, time: {}'.format(dot.pos, rospy.get_time())))
    while True:
        dot.pos = dot_pos[idx]
        dot.draw()
        if clock.getTime() > dot_duration:
            idx += 1
            clock.reset()
            win.callOnFlip(lambda: tracker.send_message('dot pos: {}, time: {}'.format(dot.pos, rospy.get_time())))

        if idx >= len(dot_pos):
            break
            # idx=0
        
        win.flip()
    
    win.close()
    tracker.stop_recording()

if __name__ == '__main__':
    main()