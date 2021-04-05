# Función auxiliar para cambiar un número N de un intervalo (a0, b0) a otro (aF, bF)
def cambiaIntervalo (N, a0, b0, aF, bF):
    return aF + (bF - aF)*(N - a0)/(b0 - a0)

def gazebo2alg (point):
    newPoint = []
    for i in range(len(point)):
        newPoint.append(cambiaIntervalo(point(i), -100, 100, 0, 31))

    return newPoint

def alg2gazebo (point):
    newPoint = []
    for i in range(len(point)):
        newPoint.append(cambiaIntervalo(point(i), 0, 31, -100, 100))

    return newPoint