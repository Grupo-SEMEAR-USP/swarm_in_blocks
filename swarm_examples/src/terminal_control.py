#!/usr/bin/python3
from time import sleep
import numpy as np
from swarm_in_blocks.swarm import Swarm
import rospy
import logging
import random

def printLogo():
    print("------------------------------- SWARM IN BLOCKS -------------------------------")
    print("                                                                               ")
    print("                                                                               ")
    print("                                                                               ")
    print("               ''''''''''''''''''''            '''''''''''''''''''''           ")
    print("             ''''''''''''''''''''''''        '''''''''''''''''''''''''         ")
    print("            ''''''''''''''''''''''''''      ''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''  ''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           ''''''''''                                                          ")
    print("           '''''''''''                                                         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("                                                            ''''''''''         ")
    print("                                                           '''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("           '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("            ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("             '''''''''''''''''''''''''''''''''''''''''''''''''''''''''         ")
    print("               ''''''''''''''''''''''''''''''''''''''''''''''''''''''          ")
    print("                                 '''''''''''''''                               ")
    print("                                   '''''''''''                                 ")
    print("                                     '''''''                                   ")
    print("                                       '''                                     ")
    print("                                                                               ")
    sleep(1)
    print("                        It's never been so easy to swarm!                      ")
    print("\n")
    sleep(1)
    print("-------------------------- Terminal Ground Control-----------------------------")
    sleep(1)
    print('\n')

# Print swarm in blocks logo
printLogo()

print("Select the operation mode:")
print("1 - Planning mode")
print("2 - Simulation mode")
print("3 - Navigation mode")
selec_mode = input('\n')

if (selec_mode == str('1')):
    # Starts the simulation just with the plots previews
    selec_amount = int(input(f"\nType the amount of clovers: "))
    swarm = Swarm(selec_amount)
    swarm.startPlanning()
elif (selec_mode == str('2')):
    print("1 - Launch simulation")
    print("2 - Simulation is already launched")
    selec_launch = int(input('\n'))
    # Starts the Gazebo simulation and clovers ready to operate
    if selec_launch == 1:
        selec_amount = int(input(f"\nType the amount of clovers: "))
        swarm = Swarm(selec_amount)
        swarm.startSimulation(launch=True)
    elif selec_launch == 2:
        swarm = Swarm()
        swarm.startSimulation()
elif (selec_mode == str('3')):
    pass
else:
    logging.debug("There isn't this mode")
    rospy.loginfo("There isn't this mode")
    exit()

# Menu
def menu():
    print("Select")
    print("\n-----Basic operations-----")
    print("1 - Takeoff all")
    print("0 - Initial position")
    print("L - Land all")
    print("RL - Return and Land")
    print("led - Set Led for all drones")
    print("\n-----Formations-----")
    print("2 - Line formation")
    print("3 - Triangle formation")
    print("4e - Empty square formation")
    print("4f - Full square formation")
    print("O - Circle formation")
    print("5 - Cube formation")
    print("6 - Sphere formation")
    print("7 - Pyramid formation")
    print("A - Alphabet formation")
    print("\n-----Transformations-----")
    print("TR - Rotate")
    print("TS - Scale")
    print("TT - Translate")
    print("\n-----Plot and Apply-----")
    print("AP - Apply formation")
    print("PLT - Plot preview")
    print("PLT3D - Plot 3D preview")
    print("FL - Formation list")
    print("\nE - Exit")

N = swarm.num_of_clovers
#init_form = swarm.setInitialPosition()

while not rospy.is_shutdown():
    coord = swarm.des_formation_coords
    menu()
    key = input('\n')
    if (key == str('1')):
        swarm.takeOffAll()
        # rospy.sleep(2)

    elif (key == str('2')):
        if (N < 2):
            print("You need at least 2 clovers!\n")
        else:
            L = int(input("Insert the desired length: "))
            swarm.setFormation2D('line', N, L)
            # rospy.sleep(5)

    elif (key == str('3')):
        if (N < 3):
            print("You need at least 3 clovers!\n")
        else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('triangle', N, L)
            # rospy.sleep(5)

    elif (key == str('4f') or key == str('4F')):
        if (N < 4):
            print("You need at least 4 clovers!\n")
        else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('full_square', N, L)
            # rospy.sleep(5)

    elif (key == str('4e') or key == str('4E')):
        if (N < 4):
            print("You need at least 4 clovers!\n")
        else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation2D('empty_square', N, L)
            # rospy.sleep(5)

    elif (key == str('o') or key == str('O')):
        L = int(input("Insert the desired ratio: "))
        swarm.setFormation2D('circle', N, L)
        # rospy.sleep(2)

    elif (key == str('5')):
        if (N < 8):
            print("You need at least 8 clovers!\n")
        else:
            #type = input("Insert full or empty: ")
            L = int(input("Insert the desired side length: "))
            swarm.setFormation3D('cube', N, L)
            # rospy.sleep(5)

    elif (key == str('6')):
        L = int(input("Insert the desired ratio: "))
        swarm.setFormation3D('sphere', N, L)
        # rospy.sleep(5)

    elif (key == str('7')):
        if (N < 3):
            print("You need at least 3 clovers!\n")
        else:
            L = int(input("Insert the desired side length: "))
            swarm.setFormation3D('pyramid', N, L)
            # rospy.sleep(5)

    elif (key == str('0')):
        swarm.returnToHome()
        rospy.sleep(2)

    elif (key == str('a') or key == str('A')):
        user_input = input(f"Please, enter word or a letter: ")
        swarm.setAlphabet(user_input, N)

    elif (key == str('l') or key == str('L')):
        swarm.landAll()
        rospy.sleep(5)

    elif (key == str('rl') or key == str('RL')):
        swarm.returnAndLand()
        rospy.sleep(5)

    elif (key == str('ts') or key == str('TS')):
        sx = int(input("Insert the x scale: "))
        sy = int(input("Insert the y scale: "))
        sz = int(input("Insert the z scale: "))
        swarm.scaleFormation(sx, sy, sz)

    elif (key == str('tr') or key == str('TR')):
        anglex = float(input("Insert the x angle: "))*np.pi/180
        angley = float(input("Insert the y angle: "))*np.pi/180
        anglez = float(input("Insert the z angle: "))*np.pi/180
        swarm.rotateFormation(anglex, angley, anglez)

    elif (key == str('tt') or key == str('TT')):
        tx = int(input("Insert the x translation: "))
        ty = int(input("Insert the y translation: "))
        tz = int(input("Insert the z translation: "))
        swarm.translateFormation(tx, ty, tz)

    elif (key == str('ciranda')):
        ang = 0
        while(ang < 4*np.pi):
            swarm.rotateFormation(0, 0, ang)
            rospy.sleep(2)
            swarm.applyFormation()
            rospy.sleep(3)

    elif (key == str('ap') or key == str('AP')):
        swarm.applyFormation()

    elif (key == str('plt') or key == str('PLT')):
        swarm.create_swarm_preview(
            swarm, swarm.des_formation_coords, preview_type='2D')

    elif (key == str('plt3d') or key == str('PLT3D') or key == str('plt3D')):
        swarm.create_swarm_preview(
            swarm, swarm.des_formation_coords, preview_type='3D')

    elif (key == str('fl') or key == str('FL')):
        print(swarm.formation_list)

    elif (key == str('Ld') or key == str('load')):
        swarm.loadFormation()

    elif (key == str('led')):
        effect = str(input("input led effect: "))
        red = int(input("Insert the red color (0-255): "))
        green = int(input("Insert the green color (0-255): "))
        blue = int(input("Insert the blue color (0-255): "))
        swarm.ledAll(effect, red, green, blue)

    elif (key == str('led2')):
        effect = str(input("input led effect: "))
        red = int(input("Insert the red color (0-255): "))
        green = int(input("Insert the green color (0-255): "))
        blue = int(input("Insert the blue color (0-255): "))
        swarm.ledEven(effect, red, green, blue)

    elif (key == str('led3')):
        strg = str(input("input formation type: "))
        effect = str(input("input led effect: "))
        print("Drones coordinates: \n{}\n".format(
            swarm.des_formation_coords))
        swarm.ledFormation2D(effect, strg, L, N)

    elif (key == str('led4')):
        effect = str(input("input led effect: "))
        red = int(input("Insert the red color (0-255): "))
        green = int(input("Insert the green color (0-255): "))
        blue = int(input("Insert the blue color (0-255): "))
        swarm.ledOdd(effect, red, green, blue)

    elif (key == str('led5')):
        #effect = str(input("input led effect: "))
        effects_list = ['fill', 'fade', 'flash', 'blink',
                        'blink_fast', 'wipe', 'rainbow', 'rainbow_fill']

        effect = effects_list[random.randint(0, 7)]
        swarm.ledRandom(effect)

    elif (key == str('e') or key == str('E')):
        break
