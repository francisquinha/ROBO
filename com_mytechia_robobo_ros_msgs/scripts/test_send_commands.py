#!/usr/bin/env python


from handler import Handler

def pausa():
    print 'Press enter to continue'
    raw_input('')


handler= Handler()

handler.send_command('LEDCOLOR', {'led': 'Front-R', 'color':'white'})
handler.send_command('LEDCOLOR', {'led': 'Front-L', 'color':'red'})
handler.send_command('LEDCOLOR', {'led': 'Front-C', 'color':'blue'})
handler.send_command('LEDCOLOR', {'led': 'Back-R', 'color':'magneta'})
handler.send_command('LEDCOLOR', {'led': 'Back-L', 'color':'green'})
handler.send_command('LEDCOLOR', {'led': 'all', 'color':'orange'})
handler.send_command('LEDCOLOR', {'led': 'all', 'color':'off'})

handler.send_command('TALK', {'text': 'hola'})
pausa()

handler.send_command('SOUND', {'sound': 'angry'})
pausa()
handler.send_command('MOVETILT', {'pos': '45', 'speed': '10'})
pausa()
handler.send_command('MOVETILT', {'pos': '90', 'speed': '5'})
pausa()
handler.send_command('MOVEPAN', {'pos': '90', 'speed': '5'})
pausa()
handler.send_command('MOVEPAN', {'pos': '180', 'speed': '5'})

#Wheel Movement
pausa()
handler.send_command('TWOWHEELSBLOCKING', {'lspeed': '50', 'rspeed': '50', 'blockid':'3', 'time':'2'})

#Ball detection settings
pausa()
handler.send_command('CONFIGUREBLOB', {'red': 'false', 'green': 'true', 'blue':'false', 'custom':'false'})












