import math
import numpy as np

op = int(input(f"Forma: \ntriangulo = 1\nPiramide = 2\n:"))
while(op!=3):
    N = int(input(f"Digite o numero de drones: "))
   

    #Formação da matriz
    matriz=[]
    for i in range(N):
        line=[]
        for j in range(4):
            line.append(0.0)
        matriz.append(line)

    if(op==1):
        #Triângulo
        L=2

        #Variáveis contadoras
        if(N<5):
            c1=1    #Primeira variável independente
        else:
            c1=1/2
        c2=0        #Segunda variável independente
        c3=1        #Parametro para base do triângulo 
        p=1         #Parametro de subtração

        id_list = []
        reta = math.sqrt(3) #Coeficiente angular

        #Laço que define as variáveis a partir do numero de drones
        for index in range(N):
            id_list.append(index)

            if(index%3==0):
                if(index>3):
                    L += 1
                    
            if(index>7):
                if(N%2!=0 or N%3==0):
                    c3 = 1/2

            if((index+1)%3==0 and index>7):
                p+=1

        #######################################
        id=int(np.median(id_list)) #Media dos ids
        h = (math.sqrt(3)*L)/2     #Altura do triângulo 

        #Verificações
        if(N%2==0 and N%3!=0):
            S=N-1

        if(N%2!=0 and N>7):
            S=N-p
        if(N%3==0):
            S=N-p

        #Coordenadas triângulo
        for c in range(0,4):
            
            for l in range(0,N):
            #Define o x     
                if(c==0): 
                    if(l<=id and reta*c1*l<=h):
                        matriz[l][c]  = reta*c1*l

                    else:
                        matriz[l][c] = reta*c1*c2
                        c2+=1
                    
                    if(l>=S and S>2):
                        matriz[l][c] = 0
                        
            #Define o y  
                elif(c==1):
                    if(l<=id and reta*c1*l<=h):
                        matriz[l][c] = c1*l
                        c2=0
                    else:
                        matriz[l][c] = L-c1*c2
                        c2+=1

                    if(l>=S):
                        matriz[l][c] = c3
                        c3+=1
            #Define o z  
                elif(c==2):
                    matriz[l][c] = 1

            #Define o quarto parametro
                elif(c==3):
                    matriz[l][c] = 1

    if(op==2):
    #Piramide 
        L=2
        for index in range(N):
            if(index%3==0):
                if(index>3):
                    L += 1

        h = (math.sqrt(3)*L)/2
        L1=L
        cp=0
        z=1
        for c in range(0,4):
            for l in range(0,N):
                if(c==0):
                    if(l%3==0):
                        L-=1/2
                    if((l-1)%3==0):
                        matriz[l][c]=round(h,2)
                    else:
                        matriz[l][c]=0
                    if(l==N-1):
                        matriz[l][c]=round(h/2, 2)
                
                if(c==1):
                    if(l%3==0 and l>0):
                        L1 -= 1/2

                    if((l-2)%3==0):
                        matriz[l][c]=L1
                    elif((l-1)%3==0):
                        matriz[l][c]=L1/2
                    else:
                        matriz[l][c]=cp
                        cp+=1/2

                    if(l==N-1):
                        matriz[l][c]=L/2

                if(c==2):

                    matriz[l][c]=z

                    if((l-2)%3==0):
                        z+=1
                
                if(c==3):
                    matriz[l][c]=1
                     
    print(matriz)
    print(id)
    op = int(input(f"Sair = 3\nFicar = 0\n:"))



