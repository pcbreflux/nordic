#!/usr/bin/php
<?php
/* Copyright (c) 2016 pcbreflux. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 */

//
// first install php: 
// sudo apt-get install php
// and use i.e. chmod 700 flash_nrf.php
//

function sendcmd($tfp,$cmd) {
	$pos=0;
	fputs($tfp,$cmd."\r");
	do {
	    $result=fread($tfp,4096); 
	    // echo "(".$pos.") [".$result,"]\n";
	    $pos++;
	} while (stripos($result,">")===false && $pos<20);
        usleep(500);  // communication bonus
	return $result;
}

// main

if ($argc!=2) {
	echo "Usage: ".$argv[0]." file.hex\n";
	exit;
}

$ip = 'localhost';
$port = '4444';
$result = '';
$telnetfp = fsockopen($ip, $port); 

if ($telnetfp) {
	stream_set_timeout($telnetfp,20);  // 20s should be enough
	//echo fgets($telnetfp); 
	$result=fread($telnetfp,1024); 
	if (stripos($result,"Open On-Chip Debugger")!==false) {
		echo "We are in!\n";
		$result=sendcmd($telnetfp,"reset halt");
		echo "reset halt ok!\n";
		$result=sendcmd($telnetfp, "program ".$argv[1]," verify");
		echo "program maybe ok!\n";
		$result=sendcmd($telnetfp,"reset");
		echo "reset ok!\n";
		$result=sendcmd($telnetfp,"exit");
		echo "exit ok!\n";
	} else {
		echo "Sorry no openocd on Port 4444!\n";
	}
	fclose($telnetfp);
} else {
	echo "Sorry no openocd on Port 4444!\n";
}
?>
