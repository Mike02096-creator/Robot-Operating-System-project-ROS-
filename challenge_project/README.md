
# Readme du challenge ROS. 

Année Universitaire 2019-2020, Deuxième semestre 
----------------------------------------------------------------------------------------------------------------------------------

Ce readme est rataché un package "challenge_project". 
Il a pour but de fournir les indications quand à l'organisation et la construction de ce package ainsi qu'à sont utilisation via les Luanch Files fournis. 
----------------------------------------------------------------------------------------------------------------------------------

Le projet auxquel ce package répond avait pour but de simuler (via Gazebo) le pilotage "automatique" d'un robot (Turtlebot 3 burger) dans différents environnements (Worlds) prévu à cet effet. 
Pour cela nous avons eu recourt au logiciel ROS et au langage python pour écrire les programmes ou "noeuds" definissant le comportement du robot dans certaines situations. 
Le projet ce decoupe en trois "Challenges" distincts chacun sous la forme d'un circuit que le robot doit parcourir avec un point de depart et d'arrivé definit. Chaque Challenge se decoupant lui même en trois tâches de difficulté croissante.

Challenge 1 : Suivis de ligne (à l'aide de la camera du robot)
Challenge 2 : Evitement d'obstacle (à l'aide du scan LIDAR du robot)
Challenge 3 : Utilisation de la camera et du scan LIDAR du robot sur un cricuit comportant des difficultés variées.   

Chaque challenge beneficie d'un Launch File qui lui est propre qui une fois lancé avec la commande `roslaunch challenge_project challengeX_taskY.launch` (X correspondant au numero du challenge et Y au numero de la tache) permet d'observer le robot simulé sur gazebo réaliser la tache demandée. 

(Note : Une vue caméra est également affichée pour chacun des challenges.)

Certain Parametres peuvent être modfifiés dans le Launch File afin de changer des details de la simulation. Ils sont indiqués par la balise `<!-- Paramètre utilisateur -->`.
----------------------------------------------------------------------------------------------------------------------------------

## Liste des Fichiers présents dans ce package : 

Outre le package.xml et le CMakeLists.txt, obligatoires dans chaque package ROS, ce dernier contient 5 Repertoires :

### 'launch' : 

Le repertoire **launch** contiens tout les launch files utilisé lors de ce projet. Il inclut bien sûr les 9 Launch Files simulant le différentes étapes du challenge mais aussi des Launch Files 'intermediaires' fournit comme base au debut du projet ou crées pour réaliser certaines actions lors de la phase d'élaboration du travail final. 

Liste des fichiers présents dans **launch** :
--------------------------------------------- 

* ***challenge1_task1.launch*** : Permet de lancer le challenge 1 Tâche 1, suivis simple d'une ligne jaune à vitesse constante fixée à 0.5. 

* ***challenge1_task2.launch*** : Permet de lancer le challenge 1 Tâche 2, suivis de ligne à vitesse variable (vitesse maximum = 0.5 pour les parties jaunes et (vitesse maximum)/2  pour les parties rouges ). 

* ***challenge1_task3.launch*** : Permet de lancer le challenge 1 Tâche 3, suivis de ligne à vitesse variable toujours en ralantissant significativement au niveau de la couleur rouge mais aussi en adaptant cette vitesse au conditions rencontrés (virage durs ou brouillage de la piste).

* ***challenge2_task1.launch*** : Permet de lancer le challenge 2 tache 1, avancer vers le mur qui se trouve en face et s'arrèter à une distance de sécurité définissable par l'utilisateur.

* ***challenge2_task2.launch*** : Permet de lancer le challenge 2 tache 2, maintenir un distance fixe avec un mur en mmouvement se trouvant en face.

* ***challenge2_task3.launch*** : Permet de lancer le challenge 2 tache 3, maintenir un distance fixe en face d'un mur en mouvement pouvant apparaitre partout autours du robot.

* ***challenge3_task1.launch*** : Permet de lancer le challenge 3 tache 1, suivis de ligne et interaction avec les portes.

* ***challenge3_task2.launch*** : Permet de lancer le challenge 3 tache 2, suivis de ligne, interaction avec les portes et passage du corridor. 

* ***challenge3_task3.launch*** : Permet de lancer le challenge 3 tache 3, suivis du circuit dans sont intégralité. 

* ***rviz.launch*** : (fourni au debut du projet) Permet de visualiser le robot dans rviz. Peut être lancé seul ou en association avec une simulation gazebo.

* ***gazebo.launch*** : (fourni au debut du projet) Permet de lancer la simulation gazebo du robot burger. Le monde utilisé est celui du challenge 1.

### 'rviz' : 

Le repertoire **rviz** contiens les fichier de configuration pour les simulations dans rviz.
Les eventuelles nouvelles configurations serons donc sauvegardées ici.

Liste des fichiers présents dans **rviz** :
--------------------------------------------- 

* ***configuration.rviz*** : Fichier utilisé dans **rviz.launch** pour configurer la simualtion du robot burger dans rviz. 

### 'scripts' :

Le repertoire **scripts** contiens tout les noeuds ROS utilisés (ou aillant été utilisés pendant le projet) dans les fichers Launch. Certains étaient fournit au debut du projet et ne sont donc pas commentés par nos soins. Tout les autrex comportent des details sur la construction du code python les composants. 

Liste des fichiers présents dans **scripts** :
----------------------------------------------

* ***challenge1_task1_linefollowing.py*** :
Code du noeud utilisé dans le challenge 1 Task 1. Ce noeud est lancé dans **challenge1_task1.launch** demande au robot de suivre la ligne jaune devant laquelle il apparaît puis de s'arrêter dans la cible verte. 
Pour cela nous utilisons les informations fournies par la caméra du robot : definition d'un subscriber au topic de la camera fournissant un message de type Image  (sensor_msgs.msg). 
Les informations sont ensuites traités, à l'aide de la librairie Open CV, dans la fonction 'callback' associée au subscriber précédement definit. 
La definition d'un publisher sur le topic comportant les instructions de vitesse du robot permet ensuite de commander l'avancé du robot.

* ***challenge1_task2_linefollowing.py*** :
Code du noeud utilisé dans le challenge 1 Task 2. Ce noeud est lancé dans **challenge1_task2.launch** demande au robot de suivre la ligne devant laquelle il apparaît en variant sa vitesse en fonction de la couleur de celle-ci (jaune : vitesse "maximum" et rouge : ralentissement d'un facteur 2) puis de s'arrêter dans la cible verte. 
Pour cela nous suivons exactement le même principe que dans le script challenge1_task1_linefollowing.py en adapatant seulement la vitesse du robot en fonction de la couleur détéctée.
La detection de la cible verte à par ailleur été ajoutée pour assurer l'arrêt du robot sur celle-ci (certaines interferences avec les lignes des autres taches du challenge ayant été remarquées). 

* ***challenge1_task3_linefollowing.py*** :
Code du noeud utilisé dans le challenge 1 Task 3. Ce noeud est lancé dans challenge1_task3.launch et demande au robot de suivre la ligne devant laquelle il apparaît en variant sa vitesse en fonction de la couleur de celle-ci (ralantissement significatif sur le rouge) mais aussi en fonction des difficultés du chemin (virages ou brouillage du circuit)  puis de s'arrêter dans la cible verte. 
Pour cela nous suivons le même principe que dans le script challenge1_task2_linefollowing.py en afinant les condtions de commandes de vitesses afin que le robot ne rique pas de se perdre. De plus un processus de recherche de ligne permet au robot de retrouver la ligne à suivre dans le cas ou il l'aurait perdu (ce phénomène est notamment observable lors d'un virage trés raide au niveau d'une piste brouillée).

* ***challenge2_task1_distanceFromWall.py*** :
Code du noeud utilisé dans le challenge 2 Task 1. Ce noeud est lancé dans challenge2_task1.launch et demande au robot d'avancer vers un mur se trouvant en face de lui puis de s'arrèter lorsque que la distance de sécuritée préalablement définie est atteinte. 
Pour cela nous utilisons les informations fournies par le laser (LIDAR) du robot : definition d'un subscriber au topic du scan fournissant un message de type LaserScan (sensor_msgs.msg). 
Les informations sont ensuites traités, à l'aide de conditions simples. 
La definition d'un publisher sur le topic comportant les instructions de vitesse du robot permet ensuite de commander l'avancé ou l'arret du robot.

* ***challenge2_task2_distanceFromWall.py*** :
Code du noeud utilisé dans le challenge 2 Task 2. Ce noeud est lancé dans challenge2_task2.launch et demande au robot de garder une distance constante au mur en mouvement se trouvant en face de lui. 
Le principe de base de ce code (publisher, subscriber ...) reste semblable au challenge 2 Task 1. Cepdendant la commande du robot n'est plus réalisée à partir de simples condtions mais d'un calcul de vitesse fonction de la distance au mur avec coéficients de type système de contrôle correcteur PID (Proportionnel, Integral, Dérivé).

* ***challenge2_task3_distanceFromWall.py*** :
Code du noeud utilisé dans le challenge 2 Task 3. Ce noeud est lancé dans challenge2_task3.launch et demande au robot de rester en face et à distance constance d'un mur pouvant surgir partout autours de lui. 
Le principe de maintiens à distance reste semblable à celui utilisé en challenge 2 Task 2 . 
De plus il a été rajouté un mécanisme permettant au robot de se tourner en face du mur dès ce dernier détécté par le LIDAR. Le fonctionnement repose sur le fait que les données du lidar sont accessibles sous la forme de distance détécté pour chaque dégré autours du robot (de 0 à 359, 0 étant le devant du robot, 90 sa gauche et ainsi de suite), la distance la plus faible est alors concidérée comme indiquant l'angle ou se trouve le mur par rapport au robot.
Les rotations sont fonctions de la valeur d'angle à couvrir pour se remettre droit.

* ***challenge3_task1_Circuit.py*** :
Code du noeud utilisé dans le challenge 3 Task 1. Ce noeud est lancé dans challenge3_task1.launch et demande au robot de 
suivre une ligne jaune. Le robot doit également s'arréter en face du premier mur à une distance suffisante en attendant son ouverture
et ensuite s'arréter de nouveau devant le mur bleu (porte de garage) pour lui envoyer un signal d'ouverture.
Le principe de base de ce code (publisher, subscriber ...) reste semblable aux challenge 1 et 2. 
Cepdendant, on ajoute un publisher 'Garage_Opener_Door' pour que le robot emette le signal d'ouverture et on utilise le scan pour que le robot s'arrête devant les obstacles.

* ***challenge3_task2_Circuit.py*** :
Code du noeud utilisé dans le challenge 3 Task 2. Ce noeud est lancé dans challenge3_task2.launch et demande au robot de réaliser le circuit proposé comprenant : suivis de ligne, attente et demande d'ouverture de porte et passage dans un corridor.
Ce code reprend tout les principes vu dans les challenges et tâches précédentes. Les principes permettant l'ouverture des portes sont détaillés dans la tache précédente de ce challenge. A cela à été ajouté un mécanisme permettant de passer a travers le corridor en utilisant le laser puis de repasser en mode camera une fois le corridor fini. 
Une large utilisation des variables globales est faite pour indiquer au robot dans quelle situation particulière il se trouve mais aussi faciliter la communication des informations entre les callabcks du scanner et de la camera.   

* ***challenge3_task3_Circuit.py*** :
Code du noeud utilisé dans le challenge 3 Task 3. Ce noeud est lancé dans challenge3_task3.launch et demande au robot de réaliser le circuit proposé comprenant : suivis de ligne, attente et demande d'ouverture de porte, passage dans un corridor et evitement d'obstacles.
Ce code reprend tout les principes vu dans les challenges et tâches précédentes. Les principes permettant l'ouverture des portes et le passage dans le corridor sont détaillés dans les taches précédentes de ce challenge. A cela à été ajouté un mécanisme additionnel permettant dans un premier temps d'identifier les croisement particuliers (bifurcation et intersection à angle droit) à l'aide de la caméra et déclanchant ainsi des comportement adapté de la part du robot afin de les négocier sans encombre. De plus une detection d'obstacle orange (via la camera) à été ajouté au processus de recherche de ligne afin que le robot fasse un choix de chemin juste et ne rencontre jamais les obstacles.
Une large utilisation des variables globales est faite pour indiquer au robot dans quelle situation particulière il se trouve mais aussi faciliter la communication des information entre les callabcks du scanner et de la camera. 

* ***challenge2_task2_world_control.py*** : (fourni au debut du projet) Noeud utilisé dans le challenge 2 task 2. Il permet de mettre en mouvement le mur.

* ***challenge2_task3_world_control.py*** : (fourni au debut du projet) Noeud utilisé dans le challenge 2 task 3. Il permet de mettre en mouvement le mur à partir d'une position aléatoire.

* ***challenge3a_world_control.py*** : (fourni au debut du projet) Noeud utilisé dans le challenge 3 task 1 (et suivantes). Il permet de controller le moivement de la première porte.

* ***challenge3b_world_control.py*** : (fourni au debut du projet) Noeud utilisé dans le challenge 3 task 1 (et suivantes). Il permet de controller le moivement de la deuxième porte.

### 'urdf' : 

Le repertoire **urdf** contiens les fichier décrivant l'architecture et les caractéristiques du robot burger utilisé dans ce challenge ainsi que d'un robot plus rudimentaire nommé 'robot_with_sensors' mis à disposition au cas ou le robot burger serais trop difficile à simuler avec la machine utilisée (non éxploité durant le challenge). 

Liste des fichiers présents dans **urdf** :
(Tout ces fichiers étaient fournis au debut du projet)
----------------------------------------------
* ***common_properties.xacro*** : Definition de propriétés xacro (couleurs) utilisés dans le fichier **turtlebot3_burger.urdf.xacro**.

* ***robot_with_sensors.gazebo*** : Definition des caractéristiques gazebo du robot_with_sensors. Inclu dans **robot_with_sensors.xacro** .

* ***robot_with_sensors.xacro*** : Definition urdf du robot_with_sensors. 

* ***robot_with_sensors_materials.xacro*** : Definition de propriétés xacro (couleurs) utilisés dans le fichier **robot_with_sensors.xacro**.

* ***turtlebot3_burger.gazebo.xacro*** :  Definition des caractéristiques gazebo du robot burger. Inclu dans **turtlebot3_burger.urdf.xacro** .

* ***turtlebot3_burger.urdf.xacro*** : Definition urdf du robot burger.

### 'worlds' : 

Le repertoire **worlds** contiens les fichiers '.world' utilisés dans les challenges et lancé dans les fichiers launch correspondants. 

Liste des fichiers présents dans **urdf** :
(Tout ces fichiers étaient fournis au debut du projet)
----------------------------------------------
* ***challenge1.world*** : Monde utilisé dans les simulations du challenge 1.

* ***challenge2.world*** : Monde utilisé dans les simulations du challenge 2.

* ***challenge3.world*** : Monde utilisé dans les simulations du challenge 3.
