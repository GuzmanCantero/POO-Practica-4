#acabado y en teoria no debe tocarse mas ya

class Horno:

    def __init__(self, numero_piezas, geom_id, temperatura_inicial = 20.0):
        self.numero_piezas = numero_piezas
        self.geom_id = geom_id
        self.temperatura = temperatura_inicial
  


    #todas estas funciones son inventadas
    def encender(self):
        pass

    def apagar(self):
        pass

    def aumentar_temperatura(self, incremento):
        self.temperatura = self.temperatura + incremento

    def reducir_temperatura(self, decremento):
        self.temperatura = self.temperatura - decremento

    def obtener_temperatura(self):
        return self.temperatura

    def actualizar_color(self, model):
        if self.temperatura < 50.0:
            color = [0.1, 0.3, 1.0, 1.0]
        elif self.temperatura < 100.0:
            color = [1.0, 0.6, 0.1, 1.0]
        else:
            color = [1.0, 0.1, 0.1, 1.0]
        
        model.geom_rgba[self.geom_id] = color