import eyelink
from psychopy import visual, core

def main(edffile='test.edf',
         screen_width=800,
         screen_height=600,
         full_screen=True,
         dot_duration=1.0):

    # create a window
    win = visual.Window(
        size=(screen_width, screen_height), fullscr=full_screen, screen=0,
        allowGUI=True, allowStencil=False,
        monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
        blendMode='avg', useFBO=False)

    # use pixels as units
    win.setUnits('pix')

    # create a visual stimulus for the dots
    dot = visual.Circle(win, radius=50.0,
                        fillColor="white",
                        lineColor="white", 
                        units='pix',
                        fillColorSpace='rgb',
                        lineColorSpace='rgb')
    margins = [0.1*screen_width, 0.1*screen_height]
    xmin = margins[0]
    xmax = screen_width - margins[0]
    ymin = margins[1]
    ymax = screen_height - margins[1]
    dot_pos = [(xmin, ymin),
               (xmin, ymax),
               (xmax, ymax),
               (xmax, ymin)]
    print(dot_pos)
    
    # keep track of time
    clock = core.Clock()

    # establish a connection to the tracker
    tracker = eyelink.Eyelink(win, edffile)

    # start the recording
    tracker.start_recording()
   
    idx = 0
    clock.reset()
    win.callOnFlip(lambda: tracker.send_message('dot pos: {}'.format(dot.pos)))
    while True:
        dot.pos = dot_pos[idx]
        dot.draw()
        if clock.getTime() > dot_duration:
            idx += 1
            clock.reset()
            win.callOnFlip(lambda: tracker.send_message('dot pos: {}'.format(dot.pos)))

        if idx >= len(dot_pos):
            break
            # idx=0
        
        win.flip()
    
    win.close()
    tracker.stop_recording()

if __name__ == '__main__':
    main()