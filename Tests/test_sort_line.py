import matplotlib.pyplot as plt
import numpy as np
from numpy import array

# Les données des pixels
# pixels = [[array([36, 47]), array([37, 47]), array([37, 46]), array([37, 45]), array([38, 44]), array([39, 44]), array([40, 45]), array([41, 45]), array([42, 46]), array([42, 45]), array([40, 44]), array([38, 43]), array([37, 44]), array([36, 48]), array([36, 49]), array([36, 50])], [array([44, 62]), array([45, 62]), array([46, 62]), array([47, 63]), array([48, 63]), array([49, 63]), array([50, 63]), array([50, 62]), array([47, 62])], [array([47, 36]), array([48, 36]), array([49, 36]), array([50, 36]), array([51, 36]), array([52, 37]), array([53, 37]), array([54, 37]), array([55, 38]), array([56, 38]), array([57, 39]), array([58, 40]), array([59, 41]), array([60, 42]), array([61, 43]), array([61, 42]), array([60, 41]), array([59, 40]), array([58, 39]), array([57, 38]), array([55, 37]), array([52, 36]), array([47, 37])], [array([59, 45]), array([60, 45])]]
# pixels_2 = [[array([36, 47]), array([42, 46]), array([38, 43]), array([36, 50])], [array([44, 62]), array([50, 63]), array([44, 62]), array([47, 63])], [array([47, 36]), array([61, 43]), array([47, 36]), array([61, 43])], [array([59, 45]), array([60, 45]), array([59, 45]), array([60, 45])]]
pixels = [[array([42, 46]), array([42, 45]), array([41, 45]), array([40, 45]), array([40, 44]), array([39, 44]), array([38, 44]), array([38, 43]), array([37, 44]), array([37, 45]), array([37, 46]), array([37, 47]), array([36, 47]), array([36, 48]), array([36, 49]), array([36, 50])], [array([44, 62]), array([45, 62]), array([46, 62]), array([47, 62]), array([47, 63]), array([48, 63]), array([49, 63]), array([50, 63]), array([50, 62])], [array([61, 43]), array([61, 42]), array([60, 42]), array([60, 41]), array([59, 41]), array([59, 40]), array([58, 40]), array([58, 39]), array([57, 39]), array([57, 38]), array([56, 38]), array([55, 38]), array([55, 37]), array([54, 37]), array([53, 37]), array([52, 37]), array([52, 36]), array([51, 36]), array([50, 36]), array([49, 36]), array([48, 36]), array([47, 36]), array([47, 37])]]
pixels_2 = [[pixels[i][len(pixels[i])//2], pixels[i][0], pixels[i][-1]] for i in range(len(pixels))]

# Liste de couleurs pour chaque groupe
colors = ['red', 'blue', 'green', 'purple', 'orange']
colors_2 = ['pink', 'cyan', 'lime', 'violet', 'moccasin']

plt.figure(figsize=(10, 10))

# Boucle pour afficher chaque groupe de pixels avec une couleur différente
for i, group in enumerate(pixels):
    x_coords = [pixel[0] + 0.5 for pixel in group]
    y_coords = [pixel[1] + 0.5 for pixel in group]
    plt.scatter(y_coords, x_coords, c=colors[i], label=f"Groupe {i + 1}", marker='s', s=100)
for i, group in enumerate(pixels_2):
    x_coords = [pixel[0] + 0.5 for pixel in group]
    y_coords = [pixel[1] + 0.5 for pixel in group]
    plt.scatter(y_coords, x_coords, c=colors_2[i], marker='s', s=100)


# Configuration de la grille
plt.gca().invert_yaxis()  # Inverser l'axe y pour avoir l'origine en haut à gauche
plt.grid(True, which='both', color='black', linestyle='-', linewidth=0.5)  # Grille noire
plt.xticks(np.arange(min(min(p[1] for p in group) for group in pixels) - 2, max(max(p[1] for p in group) for group in pixels) + 2, 1))  # Graduation des colonnes
plt.yticks(np.arange(min(min(p[0] for p in group) for group in pixels) - 2, max(max(p[0] for p in group) for group in pixels) + 2, 1))  # Graduation des lignes
plt.gca().set_aspect('equal', adjustable='box')  # Carrés parfaits pour les cases
plt.title("Affichage des pixels avec une grille 1×1")
plt.xlabel("Colonne (y)")
plt.ylabel("Ligne (x)")
plt.legend(loc='upper right')  # Ajouter une légende pour chaque groupe
plt.show()
