#!/usr/bin/python
# -*- coding: utf-8 -*-

from pyduino_pcduino import * # importe les fonctions Arduino pour Python

#import de la librairie interruption
from interruption import *

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template



# Entete declarative
codeurDroitPinA = 11 # MOSI, branché sur D2 (interruption 0)
codeurDroitPinB = 8
codeurGauchePinA = 12 # MISO, branché sur D3 (interruption 1)
codeurGauchePinB = 9

Nmoy = 40

ticksCodeurDroitTab = []
for i in range(Nmoy):
    ticksCodeurDroitTab.append(0)
ticksCodeurDroit = 0
ticksCodeurDroitPrec = 0
ticksCodeurDroitTravail = 0
indiceTicksCodeurDroit = 0

ticksCodeurGaucheTab = []
for i in range(Nmoy):
    ticksCodeurGaucheTab.append(0)
ticksCodeurGauche = 0
ticksCodeurGauchePrec = 0
ticksCodeurGaucheTravail = 0
indiceTicksCodeurGauche = 0

directionMoteurDroit = 4
pwmMoteurDroit = 5

directionMoteurGauche = 7
pwmMoteurGauche = 6

omegaDroit = 0
codeurDroitDeltaPos = 0
omegaGauche = 0
codeurGaucheDeltaPos = 0
interruptKO = False

tensionBatterie = 7.4

omega = 0

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
vref = 0. # consigne de vitesse
vrefDroit = 0. # consigne vitesse de rotation du moteur droit
vrefGauche = 0. # consigne vitesse de rotation du moteur gauche
omegaDroit = 0. # vitesse de rotation du moteur droit
omegaGauche = 0. # vitesse de rotation du moteur gauche
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
commande_avant_sat_Droit = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur droit
commande_avant_sat_Gauche = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur gauche
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
yprecDroit = 0 # mesure de la vitesse du moteur droit au calcul précédent
yprecGauche = 0 # mesure de la vitesse du moteur gauche au calcul précédent
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
P_x_Droit = 0. # valeur de l'action proportionnelle sur le moteur droit
I_x_Droit = 0. # valeur de l'action intégrale sur le moteur droit
D_x_Droit = 0. # valeur de l'action dérivée sur le moteur droit
P_x_Gauche = 0. # valeur de l'action proportionnelle sur le moteur gauche
I_x_Gauche = 0. # valeur de l'action intégrale sur le moteur gauche
D_x_Gauche = 0. # valeur de l'action dérivée sur le moteur gauche
# Variables intermédiaires
Ti = 0
Td = 0
Tt = 0
ad = 0
bd = 0
br = 0


# Variables utilisées pour les données reçues
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
Kp = 0.25 # gain proportionnel du PID
Ki = 1.5 # gain intégral du PID
Kd = 0.006 # gain dérivé du PID
moteurint = 0 # Moteur droit par défaut




T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)


#--- setup --- 
def setup():
    pinMode(codeurDroitPinA, PULLUP) # met la broche en entree avec rappel au plus actif
    pinMode(codeurDroitPinB, PULLUP) # met la broche en entree avec rappel au plus actif
    
    pinMode(codeurGauchePinA, PULLUP) # met la broche en entree avec rappel au plus actif
    pinMode(codeurGauchePinB, PULLUP) # met la broche en entree avec rappel au plus actif
    
    pinMode(directionMoteurDroit, OUTPUT)
    pinMode(pwmMoteurDroit, OUTPUT)
    
    pinMode(directionMoteurGauche, OUTPUT)
    pinMode(pwmMoteurGauche, OUTPUT)
    
    """ Pour éviter une première interruption intempestive on désactive les interruptions avant l'attachement"""
    noInterrupts()  #Inhibe les interruption
    
    attachInterrupt(0, GestionInterruptionCodeurDroitPinA, RISING)
    attachInterrupt(1, GestionInterruptionCodeurGauchePinA, RISING)
    
    interrupts() #Valide les interruptions

    CommandeMoteurDroit(0, tensionBatterie)
    CommandeMoteurGauche(0, tensionBatterie)
    

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global ticksCodeurDroit, ticksCodeurGauche, indiceTicksCodeurDroit, indiceTicksCodeurGauche, started, \
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, interruptKO, \
        tdebut, imu, ad, P_x_Droit, I_x_Droit, D_x_Droit, P_x_Gauche, I_x_Gauche, D_x_Gauche, bd, Td, Tt, yprecDroit, yprecGauche, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, ticksCodeurDroitPrec, ticksCodeurGauchePrec, \
        ticksCodeurDroitTravail, ticksCodeurGaucheTravail
        
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    # On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
    # Nombre de ticks codeur depuis la dernière fois accumulé pour les 10 derniers échantillons
    # Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurDroitPinA et GestionInterruptionCodeurDroitPinA,
    # exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
    ticksCodeurDroitTravail = ticksCodeurDroit
    ticksCodeurGaucheTravail = ticksCodeurGauche
    # Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
    ticksCodeurDroit = 0
    ticksCodeurGauche = 0

    if not interruptKO:
        if started and ((ticksCodeurDroitTravail == 0) and (ticksCodeurGaucheTravail == 0)) and (((vrefDroit != 0) and (ticksCodeurDroitTravail == 0) and abs(commandeDroit) > 2) or ((vrefGauche != 0) and (ticksCodeurGaucheTravail == 0) and abs(commandeGauche) > 2)):
            ticksCodeurDroitTravail = ticksCodeurDroitPrec
            ticksCodeurGaucheTravail = ticksCodeurGauchePrec
            for i in range(Nmoy):
                ticksCodeurDroitTab[i] = ticksCodeurDroitTab[i] + ticksCodeurDroitTravail
                ticksCodeurGaucheTab[i] = ticksCodeurGaucheTab[i] + ticksCodeurGaucheTravail
        
            # Pour l'échantillon courant, calcul de l'angle de rotation du moteur pendant la période d'échantillonnage
            codeurDroitDeltaPos = ticksCodeurDroitTab[indiceTicksCodeurDroit]
            codeurGaucheDeltaPos = ticksCodeurGaucheTab[indiceTicksCodeurGauche]
            interruptKO = True
        else:
            for i in range(Nmoy):
                ticksCodeurDroitTab[i] = ticksCodeurDroitTab[i] + ticksCodeurDroitTravail
                ticksCodeurGaucheTab[i] = ticksCodeurGaucheTab[i] + ticksCodeurGaucheTravail
        
            # Pour l'échantillon courant, calcul de l'angle de rotation du moteur pendant la période d'échantillonnage
            codeurDroitDeltaPos = ticksCodeurDroitTab[indiceTicksCodeurDroit]
            codeurGaucheDeltaPos = ticksCodeurGaucheTab[indiceTicksCodeurGauche]
            
            ticksCodeurDroitPrec = ticksCodeurDroitTravail
            ticksCodeurGauchePrec = ticksCodeurGaucheTravail
    else:
        ticksCodeurDroitTravail = ticksCodeurDroitPrec
        ticksCodeurGaucheTravail = ticksCodeurGauchePrec
        for i in range(Nmoy):
            ticksCodeurDroitTab[i] = ticksCodeurDroitTab[i] + ticksCodeurDroitTravail
            ticksCodeurGaucheTab[i] = ticksCodeurGaucheTab[i] + ticksCodeurGaucheTravail
        
        # Pour l'échantillon courant, calcul de l'angle de rotation du moteur pendant la période d'échantillonnage
        codeurDroitDeltaPos = ticksCodeurDroitTab[indiceTicksCodeurDroit]
        codeurGaucheDeltaPos = ticksCodeurGaucheTab[indiceTicksCodeurGauche]
    
    
    # Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
    ticksCodeurDroitTab[indiceTicksCodeurDroit] = 0
    ticksCodeurGaucheTab[indiceTicksCodeurGauche] = 0
    
    # Mise à jour de l'indice d'échantillon courant
    indiceTicksCodeurDroit = indiceTicksCodeurDroit + 1
    indiceTicksCodeurGauche = indiceTicksCodeurGauche + 1
    if (indiceTicksCodeurDroit == Nmoy):
        indiceTicksCodeurDroit = 0
        
    if (indiceTicksCodeurGauche == Nmoy):
        indiceTicksCodeurGauche = 0

    if (codeurDroitDeltaPos != 0) or (codeurGaucheDeltaPos != 0):
        started = True
        
    omegaDroit = -4 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 4 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    

    # Calcul du PID
    # Paramètres intermédiaires
    Ti = Ki/(Kp+0.01)
    if (Kd>0): # Si PID
        ad = Tf/(Tf+dt)
        bd = Kd/(Tf+dt)
        Td = Kp/Kd
        Tt = sqrt(Ti*Td)
    else: # Si PI
        Tt = 0.5*Ti
    
    br = dt/(Tt+0.01)

    # Terme proportionnel
    P_x_Droit = Kp * (vrefDroit - omegaDroit)
    P_x_Gauche = Kp * (vrefGauche - omegaGauche)

    # Terme dérivé
    D_x_Droit = ad * D_x_Droit - bd * (omegaDroit - yprecDroit)
    D_x_Gauche = ad * D_x_Gauche - bd * (omegaGauche - yprecGauche)

    # Calcul de la commande avant saturation
    commande_avant_sat_Droit = P_x_Droit + I_x_Droit + D_x_Droit
    commande_avant_sat_Gauche = P_x_Gauche + I_x_Gauche + D_x_Gauche

    # Application de la saturation sur la commande
    if (commande_avant_sat_Droit > umax):
        commandeDroit = umax
    elif (commande_avant_sat_Droit < umin):
        commandeDroit = umin
    else:
        commandeDroit = commande_avant_sat_Droit
    
    if (commande_avant_sat_Gauche > umax) :
        commandeGauche = umax
    elif (commande_avant_sat_Gauche < umin):
        commandeGauche = umin
    else:
        commandeGauche = commande_avant_sat_Gauche

    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x_Droit = I_x_Droit + Ki * dt * (vrefDroit - omegaDroit) + br * (commandeDroit - commande_avant_sat_Droit)
    I_x_Gauche = I_x_Gauche + Ki * dt * (vrefGauche - omegaGauche) + br * (commandeGauche - commande_avant_sat_Gauche)

    # Stockage de la mesure courant pour utilisation lors du pas d'échantillonnage suivant
    yprecDroit = omegaDroit
    yprecGauche = omegaGauche

    # Fin Calcul du PID

    CommandeMoteurDroit(commandeDroit, tensionBatterie)
    CommandeMoteurGauche(commandeGauche, tensionBatterie)
    

    
def CommandeMoteurDroit(commande, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tension = commande

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurDroit, 0)
        analogWrite(pwmMoteurDroit, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurDroit, 1)
        analogWrite(pwmMoteurDroit, -tension_int)


    
def CommandeMoteurGauche(commande, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tension = commande

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurGauche, 1)
        analogWrite(pwmMoteurGauche, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurGauche, 0)
        analogWrite(pwmMoteurGauche, -tension_int)


    
def GestionInterruptionCodeurDroitPinA():
    # Routine de service d'interruption attachée à la voie A du codeur incrémental droit
    global ticksCodeurDroit, startedDroit, omegaDroit, file8, file11, commandeDroit
    
    if not startedDroit:
        file11 = open("/sys/devices/virtual/misc/gpio/pin/gpio11", 'r') # ouvre le fichier en lecture
        file8 = open("/sys/devices/virtual/misc/gpio/pin/gpio8", 'r') # ouvre le fichier en lecture
        startedDroit = True
    else:
        # Si commandeDroit n'est pas compris entre -1.5 et 1.5, il n'y a aucun risque
        # que la vitesse ait changé de signe entre deux interruptions. On ne teste donc pas le niveau
        # de l'entrée B car ça prend trop de temps
        if ((commandeDroit < 1.5) and (commandeDroit > -1.5)) or (commandeDroit * omegaDroit < 0):
            file11.seek(0) # se place au debut du fichier
            file8.seek(0) # se place au debut du fichier
            valeurLue11 = file11.read()
            valeurLue8 = file8.read()
            
            try:
                if (valeurLue11[0] == '0' or valeurLue11[0] == '1') and (valeurLue8[0] == '0' or valeurLue8[0] == '1'):
                    valeurCodeurDroitPinA = int(valeurLue11[0])  #lit le fichier
                    valeurCodeurDroitPinB = int(valeurLue8[0])  #lit le fichier
                    
                    if valeurCodeurDroitPinA == valeurCodeurDroitPinB:
                        ticksCodeurDroit = ticksCodeurDroit + 1
                    else:
                        ticksCodeurDroit = ticksCodeurDroit - 1
            except IndexError:
                print "L'index n'existe pas"
                
        elif commandeDroit >= 1.5:
            ticksCodeurDroit = ticksCodeurDroit - 1
        elif commandeDroit <= -1.5:
            ticksCodeurDroit = ticksCodeurDroit + 1
        else:
            pass

        
def GestionInterruptionCodeurGauchePinA():
    # Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
    global ticksCodeurGauche, startedGauche, omegaGauche, file9, file12, commandeGauche
    
    if not startedGauche:
        file12 = open("/sys/devices/virtual/misc/gpio/pin/gpio12", 'r') # ouvre le fichier en lecture
        file9 = open("/sys/devices/virtual/misc/gpio/pin/gpio9", 'r') # ouvre le fichier en lecture
        startedGauche = True
    else:
        # Si commandeGauche n'est pas compris entre -1.5 et 1.5, il n'y a aucun risque
        # que la vitesse ait changé de signe entre deux interruptions. On ne teste donc pas le niveau
        # de l'entrée B car ça prend trop de temps
        if ((commandeGauche < 1.5) and (commandeGauche > -1.5)) or (commandeGauche * omegaGauche < 0):
            file12.seek(0) # se place au debut du fichier
            file9.seek(0) # se place au debut du fichier
            valeurLue12 = file12.read()
            valeurLue9 = file9.read()
            
            try:
                if (valeurLue12[0] == '0' or valeurLue12[0] == '1') and (valeurLue9[0] == '0' or valeurLue9[0] == '1'):
                    valeurCodeurGauchePinA = int(valeurLue12[0])  #lit le fichier
                    valeurCodeurGauchePinB = int(valeurLue9[0])  #lit le fichier
                    
                    if valeurCodeurGauchePinA == valeurCodeurGauchePinB:
                        ticksCodeurGauche = ticksCodeurGauche + 1
                    else:
                        ticksCodeurGauche = ticksCodeurGauche - 1
            except IndexError:
                print "L'index n'existe pas"

        elif commandeGauche >= 1.5:
            ticksCodeurGauche = ticksCodeurGauche + 1
        elif commandeGauche <= -1.5:
            ticksCodeurGauche = ticksCodeurGauche - 1
        else:
            pass

        
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, vrefDroit, vrefGauche, typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteurint
        jsonMessage = json.loads(message)
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('Kp') != None:
            Kp = float(jsonMessage.get('Kp'))
        if jsonMessage.get('Ki') != None:
            Ki = float(jsonMessage.get('Ki'))
        if jsonMessage.get('Kd') != None:
            Kd = float(jsonMessage.get('Kd'))
        if jsonMessage.get('moteurint') != None:
            moteurint = int(jsonMessage.get('moteurint'))
        
        tcourant = time.time() - T0
        # Calcul de la consigne en fonction des données reçues sur la liaison série
        if typeSignal == 0: # signal carré
            if frequence > 0:
                if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                    vref = offset + amplitude
                else:
                    vref = offset
            else:
                vref = offset + amplitude
        else: # sinus
            if frequence > 0:
                vref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
            else:
                vref = offset + amplitude

        # Application de la consigne sur chaque moteur
        if moteurint == 0:
            vrefDroit = vref
            vrefGauche = 0.
        elif moteurint == 1:
            vrefDroit = 0.
            vrefGauche = vref
        elif moteurint == 2:
            vrefDroit = vref
            vrefGauche = vref
        else:
            vrefDroit = 0.
            vrefGauche = 0.
            
        if not socketOK:
            vrefDroit = 0.
            vrefGauche = 0.
  

    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, interruptKO
        if interruptKO:
            #print "Restart interrupt"
            #print time.time() - T0
            noInterrupts()  #Inhibe les interruption
            attachInterrupt(0, GestionInterruptionCodeurDroitPinA, RISING)
            attachInterrupt(1, GestionInterruptionCodeurGauchePinA, RISING)
            interrupts() #Valide les interruptions
            interruptKO = False
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), 'Consigne':("%.2f" % vref), 'omegaDroit':("%.2f" % omegaDroit), 'omegaGauche':("%.2f" % omegaGauche), 'commandeDroit':("%.2f" % commandeDroit), 'commandeGauche':("%.2f" % commandeGauche), 'Raw':("%.2f" % tcourant) + "," + ("%.2f" % vref) + "," + ("%.2f" % omegaDroit) + "," + ("%.2f" % omegaGauche) + "," + ("%.2f" % commandeDroit) + "," + ("%.2f" % commandeGauche)})
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefDroit, vrefGauche
    print 'You pressed Ctrl+C!'
    vrefDroit = 0.
    vrefGauche = 0.
    CommandeMoteurDroit(0, 5)
    CommandeMoteurGauche(0, 5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    started = False
    startedDroit = False
    startedGauche = False
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


