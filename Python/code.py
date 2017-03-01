from Tkinter import *
from datetime import datetime, timedelta
import paho.mqtt.client as mqtt

#Clase que implementa la interfaz
class GUI:
    def __init__(self, master):
        #Inicializacion básica d ela interfaz
        self.master = master
        master.title("Dashboard Paso de Peatones")
        self.last_time_peaton=datetime.min
        self.have_peaton=False
        self.frameroot=Frame(self.master,width=800,height=600)
        self.frameroot.pack(fill="both", expand=True)
        self.frameroot.grid_propagate(False)
        #Cracion alerta de peaton
        self.text_label_peaton=StringVar()
        self.label_peaton = Label(self.frameroot, textvariable=self.text_label_peaton,font=("helvetica",14))
        self.label_peaton.grid(row=0, sticky=W+N)
        self.text_label_peaton.set("NO PEATON")
        self.label_peaton.config(fg="green")
        #Creacion selector de envio de eventos
        self.value_radio=StringVar()
        self.value_radio.set("P")
        self.radio_send_P=Radiobutton(self.frameroot,text="Peaton",variable=self.value_radio,value="P")
        self.radio_send_P.grid(row=1,sticky=W+N,pady=(10,0))
        self.radio_send_C=Radiobutton(self.frameroot,text="Coche",variable=self.value_radio,value="C")
        self.radio_send_C.grid(row=2,sticky=W+N)
        self.radio_send_N=Radiobutton(self.frameroot,text="Nada",variable=self.value_radio,value="N")
        self.radio_send_N.grid(row=3,sticky=W+N)
        #Creacion de boton para envio de eventos
        self.greet_button = Button(self.frameroot, text="Enviar", command=self.send)
        self.greet_button.grid(row=4, sticky=W+N,pady=(0,20))
        #Creacion de cuadro de texto para monitorizacion
        self.text_result=Text(self.frameroot,borderwidth=3,relief="sunken")
        self.text_result.config(font=("helvetica",9),undo=True, state=DISABLED,height=7)
        self.text_result.grid(row=5,column=0, columnspan=2
        self.scrollb=Scrollbar(self.frameroot, command=self.text_result.yview)
        self.scrollb.grid(row=5,column=1)
        self.text_result["yscrollcommand"]=self.scrollb.set
        self.text_result.config(state=NORMAL)
        self.text_result.config(state=DISABLED)
        self.text_result.see(END)

        #Inicializacion cliente MQTT
        self.client_mqtt=mqtt.Client()
        self.client_mqtt.on_connect=self.on_connect
        self.client_mqtt.on_message=self.on_message
        self.client_mqtt.connect("192.168.0.5",1883,60)

    #Funcion que es llamada cuando se produce la conexion al servidor MQTT
    def on_connect(self,client, userdata, flags, rc):
        self.text_result.config(state=NORMAL)
        self.text_result.insert(END,"Connected "+str(rc))
        self.text_result.config(state=DISABLED)
        self.text_result.see(END)
        client.subscribe("PASOPEATONES/#")

    #Funcion que es llamada cuando el cliente MQTT recibe un mensaje
    def on_message(self,client, userdata, msg):
        time=datetime.now()
        self.text_result.config(state=NORMAL)
        self.text_result.insert(END,"\n"+str(time)+" - "+msg.topic+" - "+str(msg.payload))
        self.text_result.config(state=DISABLED)
        self.text_result.see(END)
        if str(msg.payload)=="P":
            self.last_time_peaton=time
            self.have_peaton=True
            self.text_label_peaton.set("PEATON")
            self.label_peaton.config(fg="red")

    #Funcion que implementa el bucle principal del cliente MQTT
    def loop_mqtt(self):
        self.client_mqtt.loop()
        datetime_increment=self.last_time_peaton+timedelta(seconds=4)
        #Si no existe peaton, volver la interfaz a su estado de calma
        if self.have_peaton==True and datetime_increment < datetime.now():
            self.have_peaton=False
            self.text_label_peaton.set("NO PEATON")
            self.label_peaton.config(fg="green")
        #Se vuelve a ejecutar el bucle tras 250 milisegundos
        self.master.after(250,self.loop_mqtt)

    #Funcion que envia un estado cuando se produce el click en el boton correspondiente
    def send(self):
        if self.value_radio.get()=="P":
            self.client_mqtt.publish("PASOPEATONES","P")
        elif self.value_radio.get()=="C":
            self.client_mqtt.publish("PASOPEATONES","C")
        else:
            self.client_mqtt.publish("PASOPEATONES","N")

#EJECUCION PRINCIPAL
#Inicializacion de la interfaz
root = Tk()
my_gui = GUI(root)
#Ejecucion a los 250 ms del bucle principal del cliente MQTT
root.after(250,my_gui.loop_mqtt)
#Ejecucion del bucle principal de la interfaz
root.mainloop()
