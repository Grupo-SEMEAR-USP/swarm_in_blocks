## Como arrumar o plugin da camera do gazebo (se o seu erro é com o gazebo_camera_manager_plugin.cpp):

### Passo 1:
```bash
gedit /home/$USER/catkin_ws/src/sitl_gazebo/src/gazebo_camera_manager_plugin.cpp
```

### Passo 2:
Vá para a linha 763 do arquivo.

### Passo 3:
Vai ter a chamada da função mavlink_msg_storage_information_pack_chan(...).

### Passo 4:
Essa chamada está com um argumento faltando (o último).

### Passo 5: 
Coloque uma virgula na frente no argumento da linha 778. Ficando assim:
```cpp
storage_name.c_str(),                // storage name
```

### Passo 6: Pule uma linha e coloque 1 como último argumento da função. Ficando assim então:
```cpp
storage_name.c_str(),                // storage name
        1
    );
```

### Passo 7:
Salve e de catkin_make no seu ws. Provavelmente o erro no plugin da câmera estará resolvido.
