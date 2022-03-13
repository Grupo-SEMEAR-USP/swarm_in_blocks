<?php
    function php_func(){
    $command = escapeshellcmd('launch.py');
    $output = shell_exec($command);
    echo $output;
    }
?>