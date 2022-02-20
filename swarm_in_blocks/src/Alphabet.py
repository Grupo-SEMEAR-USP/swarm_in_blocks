from lib2to3.pgen2.literals import simple_escapes
import string
from tkinter.simpledialog import SimpleDialog
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



# Ideia:
# fazer um dicionário com todos os arrays das coordenadas de cada Letra
# como o dicionário associa uma chave para cada tipo de dado vai facilitar para a busca.
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
             [6, 2, z, 1],
             [6, 3, z, 1],
             [6, 5, z, 1],
             [1, 6, z, 1],
             [5, 7, z, 1]], dtype = float)  		#	Full 22

C =  np.array([[0, 2, z, 1],
              [0, 4, z, 1],
              [0, 6, z, 1],
              [2, 8, z, 1],
              [4, 8, z, 1],
              [6, 8, z, 1],
              [2, 0, z, 1],
              [4, 0, z, 1],
              [6, 0, z, 1],							#	Simple 9
              [1, 1, z, 1],
              [1, 7, z, 1],
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
             [3, 0, z, 1],
             [5, 0, z, 1],
             [1, 8, z, 1],
             [3, 8, z, 1],
             [5, 8, z, 1],
             [1, 4, z, 1],
             [3, 4, z, 1],          				#	Simple 13
             [0, 1, z, 1],
             [0, 3, z, 1],
             [0, 5, z, 1],
             [0, 7, z, 1],
             [2, 4, z, 1],          				#	Midiun 18
             [2, 0, z, 1],
             [4, 0, z, 1],
             [2, 8, z, 1],
             [4, 8, z, 1]], dtype = float)    		#   Full 22 

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
             [4, 1, z, 1],						
             [1, 1, z, 1],
             [5, 7, z, 1],
             [1, 6.5, z, 1],						#	Mediun 18
             [3, 0, z, 1],
             [3, 8, z, 1],
             [0.5, 6.5, z, 1],
             [0.5, 1.5, z, 1],
             [5.5, 1.5, z, 1],
             [1.5, 0.5, z, 1],
             [4.5, 0.5, z, 1],
             [1.5, 7.5, z, 1],
             [4.5, 7.5, z, 1],
             [5.5, 6.5, z, 1]], dtype = float) 		#	Full 28

X = np.array([[0, 0, z, 1],
             [0, 8, z, 1],
             [8, 0, z, 1],
             [8, 8, z, 1],
             [4, 4, z, 1],
             [2, 2, z, 1],
             [2, 6, z, 1],
             [6, 6, z, 1],
             [6, 2, z, 1],							#Simple 9
             [3, 5, z, 1],
             [1, 7, z, 1],
             [5, 5, z, 1],
             [7, 7, z, 1],
             [3, 3, z, 1],
             [1, 1, z, 1],
             [7, 1, z, 1],
             [5, 3, z, 1],				   			#Mediun 17
             [3.5, 4.5, z, 1],
             [4.5, 3.5, z, 1],
             [4.5, 4.5, z, 1],
             [3.5, 3.5, z, 1]], dtype = float)		#Full 21
             
Alphabet_dictionary = {
    "A": A,
    "C": C,
    "E": E,
	"O": O,
	"X": X
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
        cn = Type_Format(12, 16, 22, type)

    if(letter == 'C'):
        cn = Type_Format(9, 13, 21, type)

    if(letter == 'E'):
        cn = Type_Format(13, 18, 22, type)

    if(letter == 'O'):
        cn = Type_Format(10, 18, 28, type)

    if(letter == 'X'):
        cn = Type_Format(9, 17, 21, type)
     
    return cn 

        
def Letters_Words():
    a=1
    sum = 0
    coord = np.empty((0,4))
    fill = []
    max = []
    
    str = input(f"Please, enter a word or a letter: ")
    print(f"\nSimple: Minimum Clovers needed, for this option press S or s")
    print(f"Mediun: Average amount of clovers, for this option press M or m")
    print(f"Full: Maximum fill, for this option press F or f\n")
    
    for char in str:
        type = input(f"Please, enter fill type for {char.upper()}: ")
        sum += Letter_Verification(char.upper(), "f")
        max.append(sum)
        fill.append(Letter_Verification(char.upper(), type))
        coord = np.concatenate((coord, Alphabet_dictionary[char.upper()]))
    #print(sum)
        
            



if __name__ == "__main__":
	Letters_Words()




