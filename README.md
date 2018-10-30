# CppRobLib
Librairie de modules C++ communs aux différents projets robotique CRLG.

Cette librairie est destinée à être utilisée comme sous-module des projets MBED, Labotbox, ...

Quelques règles : 
   - Les différents modules logiciels contenus dans ce projet CppRobLib doivent être génériques, indépendant de toute plateforme matérielle ou compilateur.
   - Si une classe contient du code Qt, le fichier doit être préfixé par "Q" pour le préciser.
   - Un répertoire par module contenant les fichiers sources et headers du module.
   - Lorsqu'une version stable est livrée, un tag est posé permettant aux autres projets de pointer dessus.
   
   
