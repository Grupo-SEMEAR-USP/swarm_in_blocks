import rospy
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import traceback
import logging

sys.path.insert(0, os.path.dirname(__file__))
import transform

def plot_letter_preview(coord):
    plt.figure(figsize=(8, 8))
   # plt.figure.set_aspect('equal', adjustable='box')
    plt.plot(coord[:, 0], coord[:, 1], 'ro')
    max_point = int(np.amax(coord[:, 0:2]))
    min_point = int(np.amin(coord[:, 0:2]))
    if max_point <= 10:
        max_point = 10
    if min_point >= 0:
        min_point = 0
    plt.axis([(min_point-1), (max_point+1), (min_point-1), (max_point+1)])
    plt.xticks(np.linspace(min_point, (max_point-min_point), (max_point+1)))
    plt.yticks(np.linspace(min_point, (max_point-min_point), (max_point+1)))
    plt.grid(True)
    plt.show()

# Ideia:
# fazer um dicionário com todos os arrays das coordenadas de cada Letra
# como o dicionário associa uma chave para cada tipo de dado vai facilitar para cont busca.
# Cada letra vai ter um numero associado e para isso serão necessárias várias verificações
# para ver quantos drones serão chamados.
# Cada array letra vai ler uma quantidade da matriz master

z = 1

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
             [4, 4, z, 1],  # Simple 12
             [3, 8, z, 1],
             [1, 4, z, 1],
             [3, 4, z, 1],
             [5, 4, z, 1],  # Medium 16
             [6, 1, z, 1],
             [0, 1, z, 1],
             [0, 3, z, 1],
             [0, 5, z, 1],
             [6, 3, z, 1],
             [6, 5, z, 1],
             [0.65, 6.65, z, 1],
             [1.35, 7.35, z, 1],
             [4.85, 7.35, z, 1],
             [5.45, 6.65, z, 1]], dtype=float)  # Full 26
A_size = [12, 16, 26]

C = np.array([[0, 2, z, 1],
              [0, 4, z, 1],
              [0, 6, z, 1],
              [2, 8, z, 1],
              [4, 8, z, 1],
              [1, 7, z, 1],
              [2, 0, z, 1],
              [4, 0, z, 1],
              [1, 1, z, 1],  # Simple 9
              [6, 0, z, 1],
              [6, 8, z, 1],
              [0, 5, z, 1],
              [0, 3, z, 1],  # Medium 13
              [3, 8, z, 1],
              [5, 8, z, 1],
              [3, 0, z, 1],
              [5, 0, z, 1],
              [0.5, 6.5, z, 1],
              [0.5, 1.5, z, 1],
              [1.5, 0.5, z, 1],
              [1.5, 7.5, z, 1]], dtype=float)  # Full 21
C_size = [9, 13, 21]

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
             [2, 4, z, 1],  # Simple 13
             [0, 1, z, 1],
             [0, 3, z, 1],
             [0, 5, z, 1],
             [0, 7, z, 1],
             [4, 4, z, 1],  # Medium 18
             [5, 0, z, 1],
             [5, 8, z, 1],
             #[2, 8, z, 1],
              #[3, 8, z, 1],
              [4, 8, z, 1],
              [3, 4, z, 1],
              [4, 0, z, 1], ], dtype=float)  # Full 23
E_size = [13, 18, 23]

O = np.array([[0, 2, z, 1],
             [0, 4, z, 1],
             [0, 6, z, 1],
             [2, 8, z, 1],
             [4, 8, z, 1],
             [6, 6, z, 1],
             [6, 4, z, 1],
             [6, 2, z, 1],
             [4, 0, z, 1],
             [2, 0, z, 1],  # Simple 10
             [0, 3, z, 1],
             [0, 5, z, 1],
             [6, 5, z, 1],
             [6, 3, z, 1],
             [5, 1, z, 1],
             [1, 1, z, 1],
             [5, 7, z, 1],
             [1, 7, z, 1],  # Medium 18
             [3, 0, z, 1],
             [3, 8, z, 1],
             [0.5, 6.5, z, 1],
             [0.5, 1.5, z, 1],
             [5.5, 1.5, z, 1],
             [1.5, 0.5, z, 1],
             [4.5, 0.5, z, 1],
             [1.5, 7.5, z, 1],
             [4.5, 7.5, z, 1],
             [5.5, 6.5, z, 1]], dtype=float)  # Full 28
O_size = [10, 18, 28]

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
             [4, 8, z, 1],  # Simple 11
             [3, 0, z, 1],
             [5.7, 1, z, 1],
             [5.7, 3, z, 1],
             [4, 4, z, 1],
             [0.3, 5, z, 1],
             [0.3, 7, z, 1],  # Medium 17
             [0, 0.5, z, 1],
             [0, 0, z, 1],
             [2, 0, z, 1],
             [2, 4, z, 1],
             [3, 8, z, 1],
             [5, 8, z, 1],
             [5, 7.5, z, 1]], dtype=float)  # Full 24
S_size = [11, 17, 24]

X = np.array([[0, 0, z, 1],
             [0, 8, z, 1],
             [8, 0, z, 1],
             [8, 8, z, 1],
             [4, 4, z, 1],
             [2, 2, z, 1],
             [2, 6, z, 1],
             [6, 6, z, 1],
             [6, 2, z, 1],  # Simple 9
             [3, 5, z, 1],
             [1, 7, z, 1],
             [5, 5, z, 1],
             [7, 7, z, 1],
             [3, 3, z, 1],
             [1, 1, z, 1],
             [7, 1, z, 1],
             [5, 3, z, 1],  # Medium 17
             [3.5, 4.5, z, 1],
             [4.5, 3.5, z, 1],
             [4.5, 4.5, z, 1],
             [3.5, 3.5, z, 1]], dtype=float)  # Full 21
X_size = [9, 17, 21]

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
                    [4, 9, z, 1],  # Simple 11
                    [3.5, 1, z, 1],
                    [5, 2, z, 1],
                    [5, 4, z, 1],
                    [4, 5, z, 1],
                    [0, 6, z, 1],
                    [0, 8, z, 1],  # Medium 17
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
                    [0, 5, z, 1]], dtype=float)  # Full 31)
SWARM_S_size = [11, 17, 31]

Alphabet_dictionary = {
    "A": A,
    "C": C,
    "E": E,
    "O": O,
    "S": S,
    "X": X,
    "SWARM_S": SWARM_S
}

def Letter_Verification(letter, N):
    letter_sizes = eval(letter+'_size')
    print("S - Simple type (Need {} clovers)".format(letter_sizes[0]))
    print("M - Medium type (Need {} clovers)".format(letter_sizes[1]))
    print("F - Full type (Need {} clovers)".format(letter_sizes[2]))
    print("You have {} clovers".format(N))
    type = input(f"Select your type:")
    if((type == "S" or type == "s") and (N == letter_sizes[0])):
        return letter_sizes[0]
    elif((type == "M" or type == "m") and (N == letter_sizes[1])):
        return letter_sizes[1]
    elif((type == "F" or type == "f") and (N == letter_sizes[2])):
        return letter_sizes[2]
    else:
        logging.debug("Invalid type or size")
        rospy.loginfo("Invalid type or size")
        sys.exit()

def Letters(letter, N):
    letter_coord = Alphabet_dictionary[letter]
    #plot_letter_preview(letter_coord[:Letter_Verification(letter, type)])
    return letter_coord[:Letter_Verification(letter, N)]


def Word(word, N):
    i = 8
    list_str = list(word.upper())
    list_N = np.full(len(list_str), N, dtype=int)
    word_list = list(map(Letters, list_str, list_N))
    word_list_2 = []

    for index in range(len(word_list)):
        if(index == 0):
            word_list_2.append(word_list[index])
        else:
            word_list_2.append(transform.translateFormation(
                word_list[index], i, 0, 0))
            i += 8

    word_coord = np.vstack((word_list_2))
    return word_coord


if __name__ == "__main__":
    # coord = np.empty((0, 4))
    # str = input(f"Please, enter cont word: ")
    # coord = Word(str)

    x = Letters('C', 9)
    plot_letter_preview(x)
