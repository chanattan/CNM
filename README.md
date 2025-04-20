# Conflict Negotiation Model (CNM)
## and its comparison to ad-hoc versions of Social Force Model (SFM) and CBS-MAPF (Conflict Based Solving - Multi Agent Pathfinding)

This repository's purpose is to store and show progression of the project through a Juptyer notebook.
This research project was conducted at CITI Lab, INSA & Inria Lyon, France.

## Starter

In the folder "sources" you can find all code that is used to run CNM and its comparison.
We make use of a Jupyter notebook, the main file is called "cnm_model.ipynb" (though it has a redundant name).
The progression is linear as I added features and reworked some parts along the project's progression.
To start quickly, run the first cells and ignore the test ones until "First run". If you already know Mesa, you can run the model as it is or simply use the visualization tool developed in the cell with the same name below. There, you can choose the pre-created maps and play with CNM with different parameters, e.g., resetting the seed (using numpy random here).

## CBS-MAPF
CBS-MAPF's implementation is described in the article. However, the folder "cbs-mapf" in "source" includes basic implementation of CBS-MAPF made by Haoran Peng.
I have changed some parts of the code to have an ad-hoc implementation comparable to CNM and SFM, by changing the Space Time A* (dynamic A*) to the classic offline A* planner.
For more details please see the code or/and the article sourced below.
CBS-MAPF's actual implementation is present in the Visualization's code, as it is an offline centralized planner relying on a library the use is fairly direct.
It can be played with by switching model in the Visualization's parameters. Be aware that CBS-MAPF may take some time to run depending on the chosen map and also may provide no solution if the map is too complex, see the article for more.

## SFM
SFM is implemented at the bottom of the Jupyter notebook, it has its own Visualization tool to make things cleaner and because of its different functionalities.
The ad-hoc implementation of SFM here uses a certain threshold value to let the agents go at their next cell, if the social force computed is greater than the threshold value then the agent may move, otherwise not. Through a simple parameter exploration we have selected one of the best threshold values depending on the maps to get the best of this SFM implementation.
Please see both the code ("Evaluation" before the Visualization) and the article for more information.

# Comparison

At the bottom of the Jupyter notebook you will find both Evaluation and Experiment results cells, the first evaluates according to the defined metrics in the article. The latter compares and displays them with manual input taken from the earlier cell. More details on how the comparison is made is available in the article.

## Other
The other folders are mainly related to the article's presentation, you can check them out if you want. Some figures may be outdated, please refer to the code or the article for more reliable information.

# Related article

JFSMA 2025 - Article #13: soon to appear.

### Contact
If you have a very specific question or something you would like to discuss, you can try to reach me at chanattan[dot]sok(at)ens-rennes{dot}fr.

