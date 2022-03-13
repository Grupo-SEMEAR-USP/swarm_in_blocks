<?php
    $command = escapeshellcmd('python3 launch.py');
    $output = shell_exec($command);
    echo $output;
?>