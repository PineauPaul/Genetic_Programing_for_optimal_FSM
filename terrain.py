
from roblib import *
from scipy.ndimage import gaussian_filter

def generate_terrain_1(centre = (0,0), largeur = 20, longueur = 20, pas = 0.1, penteX = 0.2, penteY = 0.2, offset = -30,dx = 0,dy = 0):
    """Cree un relief sous-marin en forme de bol."""
    x = arange(dx, largeur+pas, pas)
    y = arange(dy, longueur+pas, pas)
    X, Y = meshgrid(x, y)
    Z = (penteX*(X-centre[0]))**2 + (penteY*(Y-centre[1]))**2
    Z = Z+offset
    #Z = np.clip(Z, None, 0)
    return X, Y, Z, (largeur, longueur)


