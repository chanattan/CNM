import numpy as np
import matplotlib.pyplot as plt

# Données récupérées : [metrique1, metrique2, ecart-type_metrique2]
data = {
    "Algo1": [[0.8, 0.6, 0.05], [0.85, 0.65, 0.04], [0.82, 0.63, 0.06]],
    "Algo2": [[0.75, 0.55, 0.07], [0.78, 0.58, 0.05], [0.76, 0.56, 0.06]],
    "Algo3": [[0.82, 0.62, 0.06], [0.83, 0.64, 0.05], [0.81, 0.61, 0.07]]
}

maps = ["Map1", "Map2", "Map3"]

# Première figure : metrique1 pour les 3 algorithmes
plt.figure(figsize=(8, 6))
for algo, values in data.items():
    metrique1 = [v[0] for v in values]
    plt.plot(maps, metrique1, label=algo, marker='o', linestyle='-')

plt.xlabel('Maps')
plt.ylabel('Metrique 1')
plt.title('Comparaison de la Metrique 1 pour les 3 algorithmes')
plt.legend()
plt.grid()
plt.show()

# Deuxième figure : metrique2 avec barres d'erreur
plt.figure(figsize=(8, 6))
for algo, values in data.items():
    metrique2 = [v[1] for v in values]
    std_dev = [v[2] for v in values]
    plt.errorbar(maps, metrique2, yerr=std_dev, label=algo, marker='o', linestyle='-', capsize=5)

plt.xlabel('Maps')
plt.ylabel('Metrique 2')
plt.title("Metrique 2 avec barres d'erreur pour chaque algorithme")
plt.legend()
plt.grid()
plt.show()