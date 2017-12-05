"""
fishspeed: 0 to 100 as a raw value
sharkspeed: arbitrary, but say 30, increasing to 100 over time. behaviour of increase can be quadratic or exponential.
wavespeed = fishspeed, calibrated
fishtailspeed = fishspeed, calibrated
fishdistance, sharkdistance = speed * Delta time, incrementing with each loop
fishposition = sharkdistance-fishdistance, calibrated

THE BASICS:
The shark chases the fish. They begin a distance, s = 20 away from each other. The shark has a speed that increases over time. The fish has a speed that is determined by the user.
They travel over a distance x=1000. If for any reason the fish gets to the same position as the shark, the shark eats the fish and you lose. If the fish gets to the maximum distance x = 1000, it escapes and you win.
The motors change their behaviour as functions of the speeds of the shark and fish.



"""

reset_button = False
target_distance = 1000 #CAN CHANGE
sharkacceleration = 5*(dt) #after 10 seconds, goes from 30 to 80. CAN CHANGE
sharkstartingdistance = -20 #CAN CHANGE

fishspeed, sharkspeed, dt, fishtailspeed, wavespeed, fishdistance, sharkdistance, stepperstepsShark, stepperstepsFish, time1, time2 = None, None, None, None, None, None, None, None, None, None, None

def setup(): # sets up the variables
    fishspeed = 0
    sharkspeed = 30 #CAN CHANGE
    dt = 0
    fishtailspeed = 0
    wavespeed = 0
    fishdistance = 0
    sharkdistance = sharkstartingdistance #CAN CHANGE
    stepperstepsShark = 0
    stepperstepsFish = 0 
    time1 = None
    time2 = None
    moveshark(0, 0)
    movefishtail(0)
    movewave(0)
    return fishspeed, sharkspeed, dt, fishtailspeed, wavespeed, fishdistance, sharkdistance, stepperstepsShark, stepperstepsFish, time1, time2

setup()

def getfishspeed(): #gets the fish speed from the accelerometer
    nonlocal fishspeed
    c = #some constant, CAN CHANGE
    
    #read the accelerometer, speed = c*0.7*xinput + 0.2*yinput + 0.1*zinput
    
    return fishspeed

def movefishposition(fishdistance, sharkdistance, stepperstepsFish): #moves the fish to a position
    c = #some constant, CAN CHANGE
    
    gap += fishdistance-sharkdistance
    currentposition = c*stepperstepsFish
    #implement stepper motor motion, using gap and currentposition. If the distance is great, make the fish position near 100 (but approaches asymptotically), and vice-versa.
    stepperstepsFish += #stepswalked
    return stepperstepsFish

def movefishtail(fishspeed): #moves the fish tail
    nonlocal fishtailspeed
    c = #some constant, CAN CHANGE
    
    fishtailspeed = c*fishspeed
    #implement servo motor motion, using fishtailspeed

def moveshark(sharkspeed, stepperstepsShark): #moves the shark
    c = #some constant, CAN CHANGE
    
    #stepperspeed = c*sharkspeed
    #implement stepper motion, using stepperspeed
    
    stepperstepsShark += #steps walked
    return stepperstepsShark

def movewave(fishspeed): #moves the waves
    nonlocal wavespeed
    c = #some constant, CAN CHANGE
    
    wavespeed = c*fishspeed
    #implement DC motor motion, using wavespeed


def sharkeat(stepperstepsShark): #shark eats fish
    moveshark(0)
    currentposition = c*stepperstepsShark
    #implement stepper motor motion, using currentposition. First move the shark to the open position.
    movefishposition(0)
    movefishtailspeed(0)
    #implement stepper motor motion, using stepperstepsShark. Then move the shark to the closed position.


    

setup()
while True:
    delaylength = 20000
    if not reset_button:
        
        if time1 and time2: #following iterations
            dt = time2 - time1
        else: #first iteration
            dt = 0
            
        time1 = #time elapsed
        
        fishspeed = getfishspeed() #get data from accelerometer, convert it a value between 0 and 100
        sharkspeed += sharkacceleration
        fishdistance += fishspeed*dt
        sharkdistance += sharkspeed*dt
        
        if stepperstepsFish*c < TOOCLOSE: #distance too close to shark
            moveshark(0, 0)
            movefishposition(1000, -20, 0)
            movefishtail(0)
            movewave(0)
            print('TOO CLOSE TO SHARK')
            reset_button = True
            delaylength = 30000
            continue
        
        if fishdistance == target_distance: #fish made it!
            fishposition = 100
            fishspeed = 100
            sharkspeed = 0
            print('YOU SURVIVED')
            reset_button = True
            delaylength = 60000
            continue
        
        elif sharkdistance > (fishdistance + 5 + sharkstartingdistance): #shark reached fish
            sharkeat(stepperstepsShark)
            print('tasty')
            reset_button = True
            delaylength = 60000
            continue
                        
        
        movefishposition(fishdistance, sharkdistance)
        movefishtail(fishspeed)
        moveshark(sharkspeed)
        movewave(fishspeed)
        
     
        
        time2 = #time elapsed
    else:
        print('PLEASE RESET FISH AND SHARK POSITIONS. Restarting in 20...')
        delay(delaylength)
        setup()
