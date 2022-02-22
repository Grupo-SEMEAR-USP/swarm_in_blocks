from lib2to3.pgen2.literals import simple_escapes
import string
from tkinter.simpledialog import SimpleDialog
from click import option
from matplotlib import projections
import mavros
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.msg import State
import time
from clover import srv
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np


def plot_letter_preview(coord):
    plt.figure(figsize=(8, 8))
    plt.figure.set_aspect('equal', adjustable='box')
    plt.plot(coord[:,0],coord[:,1],'ro')
    max_point = int(np.amax(coord[:,0:2]))
    min_point = int(np.amin(coord[:,0:2]))
    if max_point <= 10: max_point=10
    if min_point >= 0: min_point=0
    plt.axis([(min_point-1),(max_point+1),(min_point-1),(max_point+1)])
    plt.xticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.yticks(np.linspace(min_point,(max_point-min_point),(max_point+1)))
    plt.grid(True)
    plt.show()

# Ideia:
# fazer um dicionário com todos os arrays das coordenadas de cada Letra
# como o dicionário associa uma chave para cada tipo de dado vai facilitar para cont busca.
# Cada letra vai ter um numero associado e para isso serão necessárias várias verificações
# para ver quantos drones serão chamados.
# Cada array letra vai ler uma quantidade da matriz master

z=1

A = np.array([[0, 0, z, 1],
             [0, 2, z, 1],
             [0, 4, z, 1],
             [0, 6, z, 1],
             [2, 8, z, 1],
             [4, 8, z, 1],
             [6, 0, z, 1],
             [6, 2, z, 1],
             [6, 4, z, 1],
             [6, 6, z, 1],     
             [2, 4, z, 1],
             [4, 4, z, 1],           				#	Simple 12
             [3, 8, z, 1],
             [1, 4, z, 1],
             [3, 4, z, 1],
             [5, 4, z, 1],           				#	Mediun 16
             [6, 1, z, 1],
             [0, 1, z, 1],
             [0, 2, z, 1],
             [0, 3, z, 1],
             [0, 5, z, 1],
             [6, 2, z, 1],
             [6, 3, z, 1],
             [6, 5, z, 1],
             [1, 7, z, 1],
             [5, 7, z, 1]], dtype = float)  		#	Full 26

C =  np.array([[0, 2, z, 1],
              [0, 4, z, 1],
              [0, 6, z, 1],
              [2, 8, z, 1],
              [4, 8, z, 1],
              [1, 7, z, 1],
              [2, 0, z, 1],
              [4, 0, z, 1],
			  [1, 1, z, 1],			                #	Simple 9
              [6, 0, z, 1],	
              [6, 8, z, 1],
              [0, 5, z, 1],
              [0, 3, z, 1],							#	Mediun 13
              [3, 8, z, 1],
              [5, 8, z, 1],
              [3, 0, z, 1],
              [5, 0, z, 1],
              [0.5, 6.5, z, 1],
              [0.5, 1.5, z, 1],
              [1.5, 0.5, z, 1],
              [1.5, 7.5, z, 1]], dtype = float) 	#	Full 21

E = np.array([[0, 0, z, 1],
             [0, 2, z, 1],
             [0, 4, z, 1],
             [0, 6, z, 1],
             [0, 8, z, 1],
             [1, 0, z, 1],
             [2, 0, z, 1],
             [3, 0, z, 1],
             [1, 8, z, 1],
             [2, 8, z, 1],
             [3, 8, z, 1],
             [1, 4, z, 1],
             [2, 4, z, 1],          				#	Simple 13
             [0, 1, z, 1],
             [0, 3, z, 1],
             [0, 5, z, 1],
             [0, 7, z, 1],
             [2, 4, z, 1],          				#	Midiun 18
             [2, 0, z, 1],
             [3, 0, z, 1],
             [2, 8, z, 1],
             [3, 8, z, 1]], dtype = float)    		#   Full 22 

O = np.array([[0, 2, z, 1],
             [0, 4, z, 1],
             [0, 6, z, 1],
             [2, 8, z, 1],
             [4, 8, z, 1],
             [6, 6, z, 1],
             [6, 4, z, 1],
             [6, 2, z, 1],
             [4, 0, z, 1],
             [2, 0, z, 1],							#	Simple 10
             [0, 3, z, 1],
             [0, 5, z, 1],
             [6, 5, z, 1],
             [6, 3, z, 1],
             [5, 1, z, 1],						
             [1, 1, z, 1],
             [5, 7, z, 1],
             [1, 7, z, 1],						    #	Mediun 18
             [3, 0, z, 1],
             [3, 8, z, 1],
             [0.5, 6.5, z, 1],
             [0.5, 1.5, z, 1],
             [5.5, 1.5, z, 1],
             [1.5, 0.5, z, 1],
             [4.5, 0.5, z, 1],
             [1.5, 7.5, z, 1],
             [4.5, 7.5, z, 1],
             [6.5, 6.5, z, 1]], dtype = float) 		#	Full 28

S = np.array([[1, 0, z, 1],
             [4, 0, z, 1],
             [5.2, 0.4, z, 1],
             [6, 2, z, 1],
             [5, 3.7, z, 1],
             [3, 4, z, 1],
             [1, 4.3, z, 1],
             [0, 6, z, 1],
             [1, 7.7, z, 1],
             [2, 8, z, 1],
             [4, 8, z, 1],                           #   Simple 11
             [3, 0, z, 1],
             [5.7, 1, z, 1],
             [5.7, 3, z, 1],
             [4, 4, z, 1],
             [0.3, 5, z, 1],
             [0.3, 7, z, 1],                         #   Mediun 17 
             [0, 0.5, z, 1],
             [0, 0, z, 1],
             [2, 0, z, 1],
             [2, 4, z, 1],
             [3, 8, z, 1],
             [5, 8, z, 1],
             [5, 7.5, z, 1]], dtype = float)        #   Full 24

X = np.array([[0, 0, z, 1],
             [0, 8, z, 1],
             [8, 0, z, 1],
             [8, 8, z, 1],
             [4, 4, z, 1],
             [2, 2, z, 1],
             [2, 6, z, 1],
             [6, 6, z, 1],
             [6, 2, z, 1],							#   Simple 9
             [3, 5, z, 1],
             [1, 7, z, 1],
             [5, 5, z, 1],
             [7, 7, z, 1],
             [3, 3, z, 1],
             [1, 1, z, 1],
             [7, 1, z, 1],
             [5, 3, z, 1],				   			#   Mediun 17
             [3.5, 4.5, z, 1],
             [4.5, 3.5, z, 1],
             [4.5, 4.5, z, 1],
             [3.5, 3.5, z, 1]], dtype = float)		#   Full 21

SWARM_S = np.array([[1, 1, z, 1],
             [4, 1, z, 1],
             [5, 1, z, 1],
             [5, 3, z, 1],
             [5, 5, z, 1],
             [3, 5, z, 1],
             [1, 5, z, 1],
             [0, 7, z, 1],
             [1, 9, z, 1],
             [2, 9, z, 1],
             [4, 9, z, 1],                           #   Simple 11
             [3, 1, z, 1],
             [5, 2, z, 1],
             [5, 4, z, 1],
             [4, 5, z, 1],
             [0, 6, z, 1],
             [0, 8, z, 1],                         #   Mediun 17 
             [0, 1.5, z, 1],
             [0, 1, z, 1],
             [2, 1, z, 1],
             [2, 5, z, 1],
             [5, 9, z, 1],
             [5, 8.5, z, 1],
             [3, 8, z, 1],
             [2, 0, z, 1],
             [1.5, 0.5, z, 1],
             [2.5, 0.5, z, 1],
             [2.5, 8.5, z, 1],
             [3.5, 8.5, z, 1],
             [2, 0.5, z, 1],
             #[0, 9, z, 1],
             [0, 5, z, 1],
             [1, 9, z, 1]], dtype = float)        #   Full 33)
             
Alphabet_dictionary = {
    "A": A,
    "C": C,
    "E": E,
	"O": O,
    "S": S,
	"X": X,
    "SWARM_S": SWARM_S
}
def Type_Format(simple, mediun, full, type = "S"):
	if(type == "S" or type == "s"):
		return (simple)
	if(type == "M" or type == "m"):
		return (mediun)
	if(type == "F" or type == "f"):
		return full

def Letter_Verification(letter, type):
    if(letter == 'A'):
        cn = Type_Format(12, 16, 26, type)

    if(letter == 'C'):
        cn = Type_Format(9, 13, 21, type)

    if(letter == 'E'):
        cn = Type_Format(13, 18, 22, type)

    if(letter == 'O'):
        cn = Type_Format(10, 18, 28, type)
    
    if(letter == 'S'):
        cn = Type_Format(11, 17, 24, type)

    if(letter == 'X'):
        cn = Type_Format(9, 17, 21, type)

    if(letter == "SWARM_S"):
        cn = Type_Format(11, 17, 32, type)

    return cn 

        
def Letters_Words():
    cont = 1
    space = 8
    sum = 0
    index = 0
    coord = np.empty((0,4))
    letter_coord = np.empty((0,4))
    fill = []
    max = []
    
    str = input(f"Please, enter cont word or cont letter: ")
    option = input(f"\nShow one by one, pres o or O\nShow all, press a or A\n: ")
    print(f"\nSimple: Minimum Clovers needed, for this option press S or s")
    print(f"Mediun: Average amount of clovers, for this option press M or m")
    print(f"Full: Maximum fill, for this option press F or f\n")

    if(str == "SWARM_S"):
        type = input(f"Please, enter fill type for SWARM: ")
        sum += Letter_Verification("SWARM_S", "f")
        max.append(sum)
        fill.append(Letter_Verification("SWARM_S", type))
        letter_coord = Alphabet_dictionary["SWARM_S"]

    else:
        for char in str:
            type = input(f"Please, enter fill type for {char.upper()}: ")
            sum += Letter_Verification(char.upper(), "f")
            max.append(sum)
            fill.append(Letter_Verification(char.upper(), type))
            letter_coord=np.concatenate((letter_coord, Alphabet_dictionary[char.upper()]))
    
    if(option == "A" or option == "a"): 
        space = 8

    elif(option == "O" or option == "o"):
        space = 0       
    
    while(index<sum):
        
        if(len(fill)>1):
            point = [letter_coord[index][0], letter_coord[index][1], z, 1]

            if(index>=fill[cont-1] and index<=(fill[cont] + max[cont-1])):
                point = [letter_coord[index][0] + space, letter_coord[index][1], z, 1] 
            
            if(index == fill[0]-1):
                index = max[0]-1

            if(index == (fill[cont] + max[cont-1])-1):
                index = max[cont]-1
                letter_coord[index][0]+=8
                if(cont<(len(fill)-1) and len(fill)>2):
                    cont+=1
                    if(option == "A" or option == "a"):
                        space+=8
                
                if(cont==len(fill)-1):
                    index+=1
            else: 
                index+=1
    
        else:
            point = [letter_coord[index][0], letter_coord[index][1], z, 1]
            if(index == fill[0]-1):
                index = max[0]
            index+=1
        
        coord = np.concatenate((coord, [point]))

    plot_letter_preview(coord)
    #print(coord)
    return(coord)

if __name__ == "__main__":
	Letters_Words()






 


