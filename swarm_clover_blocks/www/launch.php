<?php
    // $command = escapeshellcmd('');
    error_log(print_r("TESTE",true));
    $output = shell_exec("bash /home/guisoares/.bashrc && roslaunch swarm_in_blocks swarm_main.launch num=:1");
    echo $output;
?>