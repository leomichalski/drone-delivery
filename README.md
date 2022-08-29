
#### Explicação dos arquivos
* **imav2022**: pasta do repositório.
  * **sim**: pasta das simulações.
  * **rpi4**: pasta do código que vai na raspberry 4.
    * [main.py](./main.py): arquivo que roda o programa e faz tudo funcionar em conjunto.
    * [videosource.py](./videosource.py): pega as imagens da câmera.
    * [videowebstreaming.py](./videowebstreaming.py): transmite as imagens da câmera.
    * [topics.py](./topics.py): tópicos da arquitetura "Publish/Subscribe".
    * [imageclassifier.py](./imageclassifier.py): arquivo que encapsula o classificador de imagens (supondo que haja só um classificador de imagens).
    * [categories.txt](./categories.txt): categorias do classificador de imagens (supondo que haja só um classificador de imagens).
    * [requirements.txt](./requirements.txt): arquivo com as versões das bibliotecas Python utilizadas.
    * [utils.py](./utils.py): métodos aleatórios que ajudam em algo (tipo um "miscellaneous").
