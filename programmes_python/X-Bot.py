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

Nmoy = 10

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
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
xidotmes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = 1 # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = 10 # gain intégral pour l'asservissement de vitesse longitudinale
Kdvx = 0.00 # gain dérivé pour l'asservissement de vitesse longitudinale
Kpxidot = 0.1 # gain proportionnel pour l'asservissement de rotation
Kixidot = 1 # gain intégral pour l'asservissement de rotation
Kdxidot = 0.000 # gain dérivé pour l'asservissement de rotation
commande_avant_sat_vx = 0. # commande avant la saturation pour l'asservissement de vitesse longitudinale
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_avant_sat_xidot = 0. # commande avant la saturation pour l'asservissement de rotation
commande_xidot = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
D_vx = 0. # action dérivée pour l'asservissement de vitesse longitudinale
P_xidot = 0. # action proportionnelle pour l'asservissement de rotation
I_xidot = 0. # action intégrale pour l'asservissement de rotation
D_xidot = 0. # action dérivée pour l'asservissement de rotation
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
yprecvx = 0. # Mesure de la vitesse longitudinale au calcul précédent
yprecxidot = 0. # Mesure de la vitesse de rotation au calcul précédent
# Variables intermédiaires
Ti = 0
ad = 0
bd = 0

# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
Kp2 = 1.
Ki2 = 1.
Kd2 = 1.
Kpxi2 = 1.
Kixi2 = 1.
Kdxi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xidotref = 0.
vref = 0.
psidotref = 0.


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
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, \
        tdebut, ad, P_vx, I_vx, D_vx, P_xidot, I_xidot, D_xidot, bd, Ti, yprecvx, yprecxidot, interruptKO, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, xidotmes, vxref, xidotref, \
        ticksCodeurDroitPrec, ticksCodeurGauchePrec, ticksCodeurDroitTravail, ticksCodeurGaucheTravail
        
    tdebut = time.time()
        
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
        if started and ((vxref != 0) or (xidotref != 0)) and ((ticksCodeurDroitTravail == 0) and (ticksCodeurGaucheTravail == 0)) and (((ticksCodeurDroitTravail == 0) and abs(commandeDroit) > 2) or ((ticksCodeurGaucheTravail == 0) and abs(commandeGauche) > 2)):
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
    

    # Application de la consigne lue
    vxref = x1
    xidotref = x2

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2
    xidotmes = -(omegaDroit - omegaGauche)*R/W

    # Calcul du PID sur vx
    # Paramètres intermédiaires
    Ti = Ki2 * Kivx/(Kp2 * Kpvx + 0.01)
    ad = Tf/(Tf+dt)
    bd = Kd2 * Kdvx/(Tf+dt)
    
    # Terme proportionnel
    P_vx = Kpvx * Kp2 * (vxref - vxmes)

    # Terme dérivé
    D_vx = ad * D_vx - bd * (vxmes - yprecvx)
    
    # Calcul de la commande
    commande_vx = P_vx + I_vx


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * Ki2 * dt * (vxref - vxmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecvx = vxmes
    
    # Fin Calcul du PID sur vx

    # Calcul du PID sur xidot
    # Paramètres intermédiaires
    Ti = Kixi2 * Kixidot/(Kpxi2 * Kpxidot + 0.01)
    ad = Tf/(Tf+dt)
    bd = Kdxi2 * Kdxidot/(Tf+dt)
    
    # Terme proportionnel
    P_xidot = Kpxidot * Kpxi2 * (xidotref - xidotmes)

    # Terme dérivé
    D_xidot = ad * D_xidot - bd * (xidotmes - yprecxidot)
    
    # Calcul de la commande
    commande_xidot = P_xidot + I_xidot + D_xidot


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xidot = I_xidot + Kixidot * Kixi2 * dt * (xidotref - xidotmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecxidot = xidotmes
    
    # Fin Calcul du PID sur xidot


    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx - commande_xidot);
    commandeGauche = (commande_vx + commande_xidot);
      
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
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    
    def on_message(self, message):
        global x1, x2, Kp2, Ki2, Kd2, Kpxi2, Kixi2, Kdxi2

        jsonMessage = json.loads(message)
        
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref'))
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('psidotref') != None:
            x2 = (float(jsonMessage.get('psidotref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('Kp2ref') != None:
            Kp2 = float(jsonMessage.get('Kp2ref'))
            #print ("Kp2: %.2f" % Kp2)
        if jsonMessage.get('Ki2ref') != None:
            Ki2 = float(jsonMessage.get('Ki2ref'))
            #print ("Ki2: %.2f" % Ki2)
        if jsonMessage.get('Kd2ref') != None:
            Kd2 = float(jsonMessage.get('Kd2ref'))
            #print ("Kd2: %.2f" % Kd2)
        if jsonMessage.get('Kpxi2ref') != None:
            Kpxi2 = float(jsonMessage.get('Kpxi2ref'))
            #print ("Kpxi2: %.2f" % Kpxi2)
        if jsonMessage.get('Kixi2ref') != None:
            Kixi2 = float(jsonMessage.get('Kixi2ref'))
            #print ("Kixi2: %.2f" % Kixi2)
        if jsonMessage.get('Kdxi2ref') != None:
            Kdxi2 = float(jsonMessage.get('Kdxi2ref'))
            #print ("Kdxi2: %.2f" % Kdxi2)
        

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xidotref, vxmes, xidotmes, interruptKO

        if interruptKO:
            #print "Restart interrupt"
            #print time.time() - T0
            noInterrupts()  #Inhibe les interruption
            attachInterrupt(0, GestionInterruptionCodeurDroitPinA, RISING)
            attachInterrupt(1, GestionInterruptionCodeurGauchePinA, RISING)
            interrupts() #Valide les interruptions
            interruptKO = False
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), 'Consigne vitesse longitudinale':("%.2f" % x1), 'Consigne vitesse de rotation':("%.2f" % x2), 'Vitesse longitudinale':("%.2f" % vxmes), 'Vitesse de rotation':("%.2f" % (180 * xidotmes/3.141592)), 'omegaDroit':("%.2f" % omegaDroit), 'omegaGauche':("%.2f" % omegaGauche), 'commandeDroit':("%.2f" % commandeDroit), 'commandeGauche':("%.2f" % commandeGauche), 'Raw':("%.2f" % tcourant) + "," + ("%.2f" % x1) + "," + ("%.2f" % x2) + "," + ("%.2f" % vxmes) + "," + ("%.2f" % (180 * xidotmes/3.141592)) + "," + ("%.2f" % omegaDroit) + "," + ("%.2f" % omegaGauche) + "," + ("%.2f" % commandeDroit) + "," + ("%.2f" % commandeGauche)})
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
    global commandeDroit, commandeGauche
    print 'You pressed Ctrl+C!'
    commandeDroit = 0.
    commandeGauche = 0.
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


