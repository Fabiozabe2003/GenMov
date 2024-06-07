class Equipo():
    puntos=0
    goles_af=0
    goles_ec=0
    dif_g=0

    def __init__(self,nombre):
        self.nombre=nombre

    def dg(self):
        self.dif_g=self.goles_af-self.goles_ec
    
    def resultado(self,gaf,gec):
        self.goles_af+=gaf
        self.goles_ec+=gec
        self.dg()
        if(gaf>gec):
            self.puntos+=3
        elif(gaf==gec):self.puntos+=1
Alianza=Equipo("Alianza")
Fluminense=Equipo("Fluminense")
ColoColo=Equipo("Colo Colo")
CerroPorteno=Equipo("Cerro Porteño")
grupoA=[Fluminense,CerroPorteno,Alianza,ColoColo]
def evaluar_equipo(grupo,equipo,goles,LoV):
    if(grupo[0].nombre==equipo):
        if LoV=="L":
            grupo[0].resultado(goles[0],goles[1])
        else:
            grupo[0].resultado(goles[1],goles[0])
    
    elif(grupo[1].nombre==equipo):
        if LoV=="L":
            grupo[1].resultado(goles[0],goles[1])
        else:
            grupo[1].resultado(goles[1],goles[0])
    
    elif(grupo[2].nombre==equipo):
        if LoV=="L":
            grupo[2].resultado(goles[0],goles[1])
        else:
            grupo[2].resultado(goles[1],goles[0])
    
    elif(grupo[3].nombre==equipo):
        if LoV=="L":
            grupo[3].resultado(goles[0],goles[1])
        else:
            grupo[3].resultado(goles[1],goles[0])
def cabecera_grupo(letra):
    print("\n\t\tGrupo {}".format(letra))
    print("------------------------------------------")
    print("Equipo\t\tPts\t+-\t+\t-")
def grupo_ordenado(grupo):
    grupo = sorted(grupo, key=lambda grupo:(grupo.puntos, grupo.dif_g, grupo.goles_af), reverse=True)
    print(grupo[0].nombre,"\t",grupo[0].puntos,"\t",grupo[0].dif_g,"\t",grupo[0].goles_af,"\t",grupo[0].goles_ec)
    print(grupo[1].nombre,"\t",grupo[1].puntos,"\t",grupo[1].dif_g,"\t",grupo[1].goles_af,"\t",grupo[1].goles_ec)
    print(grupo[2].nombre,"\t",grupo[2].puntos,"\t",grupo[2].dif_g,"\t",grupo[2].goles_af,"\t",grupo[2].goles_ec)
    print(grupo[3].nombre,"\t",grupo[3].puntos,"\t",grupo[3].dif_g,"\t",grupo[3].goles_af,"\t",grupo[3].goles_ec)
def formato_partidos(grupo):
    partidos=[
        "{}-{}".format(grupo[2].nombre,grupo[0].nombre),"{}-{}".format(grupo[3].nombre,grupo[1].nombre), 
        "{}-{}".format(grupo[0].nombre,grupo[3].nombre),"{}-{}".format(grupo[1].nombre,grupo[2].nombre),
        "{}-{}".format(grupo[3].nombre,grupo[2].nombre),"{}-{}".format(grupo[1].nombre,grupo[0].nombre),
        "{}-{}".format(grupo[2].nombre,grupo[1].nombre), "{}-{}".format(grupo[3].nombre,grupo[0].nombre),
        "{}-{}".format(grupo[2].nombre,grupo[3].nombre),"{}-{}".format(grupo[0].nombre,grupo[1].nombre),
        "{}-{}".format(grupo[1].nombre,grupo[3].nombre), "{}-{}".format(grupo[0].nombre,grupo[2].nombre)] 
    return partidos
def grupo_resultados(grupo,letra):
    c=0
    partidos=formato_partidos(grupo)
    for i in partidos:
        if(c%2==0 and c!=0):
            print("Fin fecha {}".format(int(c/2)))
            cabecera_grupo(letra)
            grupo_ordenado(grupo)     
        print("\n",i)
        equipos=i.split("-")
        resultado=input("Resultado del partido\nPoner en formato ({}-{}): ".format(equipos[0],equipos[1]))
        print(resultado)
        goles=resultado.split("-")
        goles_l=int(goles[0])
        goles_v=int(goles[1])
        goles=[goles_l,goles_v]

        evaluar_equipo(grupo,equipos[0],goles,"L")
        evaluar_equipo(grupo,equipos[1],goles,"V")
        c+=1
    return grupo

grupoA=grupo_resultados(grupoA,"A")
print("\n\n")
cabecera_grupo("A")
grupo_ordenado(grupoA)