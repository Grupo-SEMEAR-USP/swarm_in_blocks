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
import transform


def plot_letter_preview(coord):
    plt.figure(figsize=(8, 8))
   # plt.figure.set_aspect('equal', adjustable='box')
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
             [3, 8, z, 1],
             [4, 8, z, 1],
             [3, 4, z, 1],
             [4, 0, z, 1],], dtype = float)    		#   Full 25 

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
             [5.5, 6.5, z, 1]], dtype = float) 		#	Full 28

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
             [4.5, 1, z, 1],
             #[5, 1, z, 1],
             [5, 3, z, 1],
             [5, 5, z, 1],
             [3, 5, z, 1],
             [1, 5, z, 1],
             [0, 7, z, 1],
             [0.5, 9, z, 1],
             [1.5, 9, z, 1],
             [1.5, 9, z, 1],
             [4, 9, z, 1],                           #   Simple 11
             [3.5, 1, z, 1],
             [5, 2, z, 1],
             [5, 4, z, 1],
             [4, 5, z, 1],
             [0, 6, z, 1],
             [0, 8, z, 1],                         #   Mediun 17 
             [0, 1.5, z, 1],
             [0, 1, z, 1],
             [2.25, 1, z, 1],
             [2, 5, z, 1],
             [5, 9, z, 1],
             [5, 8.5, z, 1],
             [2.75, 8, z, 1],
             [2.25, 0, z, 1],
             [1.625, 0.5, z, 1],
             [2.875, 0.5, z, 1],
             [2.125, 8.5, z, 1],
             [3.375, 8.5, z, 1],
             [2.25, 0.5, z, 1],
             #[0, 9, z, 1],
             [0, 5, z, 1]], dtype = float)        #   Full 31)
             
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

def Letter_Verification(letter):
    type = input(f"Please press the type: ")

    if(letter == 'A'):
        cn = Type_Format(12, 16, 26, type)

    if(letter == 'C'):
        cn = Type_Format(9, 13, 21, type)

    if(letter == 'E'):
        cn = Type_Format(13, 18, 25, type)

    if(letter == 'O'):
        cn = Type_Format(10, 18, 28, type)
    
    if(letter == 'S'):
        cn = Type_Format(11, 17, 24, type)

    if(letter == 'X'):
        cn = Type_Format(9, 17, 21, type)

    if(letter == "SWARM_S"):
        cn = Type_Format(11, 17, 31, type)

    return cn 
   
def Letters(letter):
    letter_coord = Alphabet_dictionary[letter]
    #plot_letter_preview(letter_coord[:Letter_Verification(letter, type)])
    return letter_coord[:Letter_Verification(letter)]

def Word(list_str):
   i=6 
   word_list = list(map(Letters, list_str))
   word_list_2=[]

   for index in range(len(word_list)):
        if(index==0):
            word_list_2.append(word_list[index])
        else:
            word_list_2.append(transform.translateFormation(word_list[index],i,0,0))
            i+=8

   word_coord = np.vstack((word_list_2))
   return word_coord
     
if __name__ == "__main__":
    coord = np.empty((0,4))
    #str = input(f"Please, enter cont word: ")
    #list_str = list(str.upper())
    #Word(list_str)
    coord = Letters('SWARM_S')
    plot_letter_preview(coord)
    
   

    




 


