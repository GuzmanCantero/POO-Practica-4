from src.horno import Horno
from src.planta import PlantaIndustrial
from src.cinta_transportadora import CintaTransportadora
from src.camara_inspeccion import CamaraInspeccion
#from src.horno import Horno
from src.objeto_industrial import ObjetoIndustrial
from src.mujoco import MuJoCo
from src.robot import LoadMenagerie



def main():
    horno = Horno(1,1,100)
    load_manager = LoadMenagerie()
    load_manager.find_franka_model()

    planta = PlantaIndustrial()
    planta.ejecutar()

if __name__ == "__main__":
    main()