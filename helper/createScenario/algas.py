import random
import numpy as np

# random.seed(42) # so usar essa linha em testes

def inputAlgas(i, x, y, value):
  model = "<model name='alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x) + " " + str(y) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n"

  return model

def inputConjuntoAlgas(i, x, y, value):
  model = "<model name='Conjunto1alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x) + " " + str(y) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto2alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x+value) + " " + str(y) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto3alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x+value) + " " + str(y+value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto4alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x+value) + " " + str(y-value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto5alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x) + " " + str(y+value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto6alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x) + " " + str(y-value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto7alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x-value) + " " + str(y) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto8alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x-value) + " " + str(y+value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto9alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x-value) + " " + str(y-value) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto10alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x-(value*2)) + " " + str(y+(value/2)) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n\
            <model name='Conjunto10alga" + str(i) + "'>\n\
            <include>\n\
              <pose>" + str(x-(value*2)) + " " + str(y-(value/2)) + " 0 0 0 3.1416</pose>\n\
              <uri>model://algas</uri>\n\
            </include>\n\
          </model>\n"

  return model

# Superior esquerda 140 -550
# Superior direita -150 -120
# Inferior esquerda 440 -400 
# Inferior direita 172 100

xx, yy = [], []
for i in range(25, 125): # inicio X e final X
    for j in range(140, 240): # inicio Y e final Y
      if random.random() < 0.0009: # intensidade das algas
        xx.append(i)
        yy.append(j)

value = 1

cx, cy = [], []
# # gera arquivo txt com os modelos para por no .world, considerando apenas uma altura
f = open("algas1.txt", "w")
for i in range(len(xx)):
  if random.random() < 0.6:
    f.write(inputAlgas(str(i) + "c1", xx[i], yy[i], value))
    f.write(inputAlgas(str(i) + "c2", xx[i]+(value*4), yy[i], value))
    f.write(inputAlgas(str(i) + "c3", xx[i]+(value*4), yy[i]+(value*4), value))
    f.write(inputAlgas(str(i) + "c4", xx[i], yy[i]+(value*4), value))
    # f.write(inputAlgas(str(i) + "c5", xx[i], yy[i]-(value*4), value))
    cx.append(xx[i])
    cy.append(yy[i])
  else:
    f.write(inputAlgas(i, xx[i], yy[i], value))

f.close()

import matplotlib.pyplot as plt
plt.xlim()
plt.plot(xx, yy, ".b")
plt.plot(cx, cy, ".r")
plt.show()
