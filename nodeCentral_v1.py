#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import rospy
from std_msgs.msg import Int32, Float32MultiArray

from base_node import BaseNode   # ajusta la ruta si el fichero se llama distinto

class CentralNode(BaseNode):

    def __init__(self):
        # Llama al constructor de BaseNode
        super(CentralNode, self).__init__(
            name="central_node",
            param_namespace="~",   # parámetros privados de este nodo
            package_path="",
            clear_params_on_exit=False
        )

        # Estado interno
        self.juego_actual = None
        self.ganador_actual = None
        self.lista_cubos = []
        self.lista_fichas = []

    #----------------- PUBLISHERS -----------------
    def init_publishers(self):
        # Más adelante: publicar objetivo del robot, etc.
        pass

    #----------------- SUBSCRIBERS ----------------
    def init_subscribers(self):
        self.sub_juego = rospy.Subscriber(
            "/juego", Int32, self.callback_juego
        )
        self.sub_ganador = rospy.Subscriber(
            "/ganador", Int32, self.callback_ganador
        )
        self.sub_posicion_cubos = rospy.Subscriber(
            "/posicioncubos", Float32MultiArray, self.callback_posicion_cubos
        )
        self.sub_posicion_fichas = rospy.Subscriber(
            "/posicionfichas", Float32MultiArray, self.callback_posicion_fichas
        )
        self.loginfo("Suscrito a /juego, /ganador, /posicioncubos y /posicionfichas", force=True)

    #----------------- LOOP ------------------------
    def loop(self):
        # De momento, todo va por callbacks
        pass

    #----------------- EVENTOS ---------------------
    def on_reload(self):
        self.loginfo("Parámetros recargados", force=True)

    def on_close(self):
        self.logwarn("Cerrando CentralNode...", force=True)

    #----------------- CALLBACKS -------------------
    def callback_juego(self, msg):
        self.juego_actual = msg.data
        self.loginfo(f"Recibido /juego = {self.juego_actual}", force=True)

    def callback_ganador(self, msg):
        self.ganador_actual = msg.data
        self.loginfo(f"Recibido /ganador = {self.ganador_actual}", force=True)
        # aquí luego llamaremos a procesar_jugada()

    def callback_posicion_cubos(self, msg):
        datos = list(msg.data)
        self.lista_cubos = self._multiarray_to_list_of_lists(datos, fields_per_element=3)
        self.loginfo(f"/posicioncubos: {len(self.lista_cubos)} cubos", force=True)

    def callback_posicion_fichas(self, msg):
        datos = list(msg.data)
        self.lista_fichas = self._multiarray_to_list_of_lists(datos, fields_per_element=3)
        self.loginfo(f"/posicionfichas: {len(self.lista_fichas)} fichas", force=True)

    def _multiarray_to_list_of_lists(self, data, fields_per_element=3):
        if len(data) == 0:
            return []
        if len(data) % fields_per_element != 0:
            self.logwarn(
                f"len(data)={len(data)} no es múltiplo de {fields_per_element}",
                force=True
            )
            return [data]
        lista = []
        num = len(data) // fields_per_element
        for i in range(num):
            start = i * fields_per_element
            end = start + fields_per_element
            lista.append(data[start:end])
        return lista


if __name__ == "__main__":
    node = CentralNode()
    node.start_spin()   # o node.start_loop() si usas 'rate'
