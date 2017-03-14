#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Fonctions de gestion des interruptions basées sur le code de cwilt du forum PCDuino (nommage 
	compatible Arduino et suppression des prints en cas d'erreur)
	Frank SAURET 2015 - www.electropol.fr
	licence creative common NC-by-sa
"""
""" Arduino UNO permet l'utilisation de 2 interruptions nommées Int0 et Int1 respectivement 
	disponibles	sur les broches 2 et 3. Pour activer ces interruptions sur la PCDuino il faut faire ce qui suit :
		sudo modprobe sw_interrupt
		echo sw_interrupt | sudo tee -a /etc/modules
		
		puis ajouter 
			KERNEL=="swirq", MODE="0666" 
		à la fin du fichier /etc/udev/rules.d/99-local.rules en faisant 		
			sudo geany /etc/udev/rules.d/99-local.rules 
"""
from fcntl import ioctl
from os import getpid
import signal
import struct
#from pyduino_pcduino import *
#import threading

SWIRQ_START=0X201
SWIRQ_STOP=0X202
SWIRQ_SETPID=0X203
SWIRQ_ENABLE=0X204
SWIRQ_DISABLE=0X205

SWIRQ_RISING=0X0
SWIRQ_FALLING=0X1
SWIRQ_HIGH=0X2
SWIRQ_LOW=0X3
SWIRQ_CHANGE=0X4

LOW =  0
HIGH = 1
RISING=2
FALLING=3
CHANGE=4

SWIRQ_PATH='/dev/swirq'

def interrupts():
	"""
		En - Re-enables interrupts [after they've been disabled by noInterrupts()] 
		
		Fr - Revalide les interruptions [après qu'elles aient été désactivées par noInterrupts()] 
	"""
	fichier=open(SWIRQ_PATH,'r')
	irqnum=0x0
	ret=ioctl(fichier,SWIRQ_ENABLE,struct.pack("@B",irqnum))
	irqnum=0x1
	ret=ioctl(fichier,SWIRQ_ENABLE,struct.pack("@B",irqnum))
	fichier.close

def attachInterrupt(interrupt,ISR,mode):
	"""
		En - Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs. Replaces any
		previous function that was attached to the interrupt. PCDuino boards have two external 
		interrupts : numbers 0 (on digital pin 2) and 1 (on digital pin 3). 
		
		Fr - Définis une fonction d'interruption  (ISR) qui sera appelée quand une interruption se produira. 
		Cette définition remplace toute ancienne fonction préalablement attachée à cette interruption.
		La PCDuino a 2 interruptions externes : 0 sur la patte 2 et 1 sur la patte 3.
		--------------------------------------------
		En -  Keyword arguments:
		interrupt -- the number of the interrupt (0 or 1)
		ISR -- the ISR (named Interrupt Service Routine ) to call when the interrupt occurs ; this 
			function must take no parameters and return nothing. This function is sometimes referred to as an
			interrupt service routine.
		mode -- defines when the interrupt should be triggered. 
			Five constants are predefined as valid values :
				- LOW to trigger the interrupt whenever the pin is low,
				- CHANGE to trigger the interrupt when the pin changes value,
				- RISING to trigger when the pin goes from low to high,
				- FALLING for when the pin goes from high to low,
				- HIGH to trigger the interrupt whenever the pin is high.	
		
		Fr - Paramètres nommés :
		interrupt -- le numéro de l'interruption (0 ou 1)
		ISR -- Le nom de la fonction qui sera appelé quand l'interruption se produira ; cette fonction ne reçoit 
		aucun paramètre et ne retourne rien. 
		mode -- Définis ce qui déclenchera l'interruption. 
			Cinq constantes permettent ce choix :
				- LOW l'interruption se produira tant que l'entrée sera à un niveau bas.
				- CHANGE l'interruption se produira à chaque changement sur l'entrée,
				- RISING l'interruption se produira à chaque front montant sur l'entrée,
				- FALLING l'interruption se produira à chaque front descendant sur l'entrée,
				- HIGH l'interruption se produira tant que l'entrée sera à un niveau haut.	
	"""
	if mode==LOW:
		hwmode=SWIRQ_LOW
	elif mode==FALLING:
		hwmode=SWIRQ_FALLING
	elif mode==RISING:
		hwmode=SWIRQ_RISING
	elif mode==CHANGE:
		hwmode=SWIRQ_CHANGE
	elif mode==HIGH:
		hwmode=SWIRQ_HIGH

	if interrupt==0:
		#signal.signal(signal.SIGUSR1,ISR)
		signal.signal(signal.SIGUSR1, lambda a, b: ISR())
	elif interrupt==1:
		#signal.signal(signal.SIGUSR2,ISR)
		signal.signal(signal.SIGUSR2, lambda a, b: ISR())
		
	mypid=getpid()
	irqnum=interrupt
	fichier=open(SWIRQ_PATH,'r')
	ret=ioctl(fichier,SWIRQ_STOP,struct.pack("@B",irqnum))
	ret=ioctl(fichier,SWIRQ_SETPID,struct.pack("@Bii",irqnum,hwmode,mypid))
	ret=ioctl(fichier,SWIRQ_START,struct.pack("@B",irqnum))
	fichier.close
	
def detachInterrupt(interrupt):
	"""
		En - Turns off the given interrupt.
		Fr - Désactive l'interruption.
		-----------------------------
		En -  Keyword arguments:
		interrupt: the number of the interrupt to disable (0 or 1)
		
		Fr - Paramètres nommés :		 
		interrupt : le numéro de l'interruption à désactiver (0 ou 1)
	"""	
	irqnum=interrupt
	fichier=open(SWIRQ_PATH,'r')
	ret=ioctl(fichier,SWIRQ_STOP,struct.pack("@B",irqnum))
	fichier.close
	
def noInterrupts():
	"""
		En - Disables interrupts [you can re-enable them with interrupts()]. 
		
		Fr - Désactive les interruptions [vous pouvez les réactiver avec interrupts()]
	"""
	fichier=open(SWIRQ_PATH,'r')
	irqnum=0x0
	ret=ioctl(fichier,SWIRQ_DISABLE,struct.pack("@B",irqnum))
	irqnum=0x1
	ret=ioctl(fichier,SWIRQ_DISABLE,struct.pack("@B",irqnum))
	fichier.close
