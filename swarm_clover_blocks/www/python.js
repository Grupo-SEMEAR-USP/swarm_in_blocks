/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

import {params} from './ros.js';

// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('_b,_print');
Blockly.Python.addReservedWords('rospy,srv,Trigger,get_telemetry,navigate,set_velocity,land');
Blockly.Python.addReservedWords('navigate_wait,land_wait,wait_arrival,wait_yaw,get_distance');
Blockly.Python.addReservedWords('pigpio,pi,Range');
Blockly.Python.addReservedWords('SetLEDEffect,set_effect,led_count,get_led_count');
Blockly.Python.addReservedWords('SetLEDs,LEDState,set_leds');

const IMPORT_SRV = `from clover import srv
from std_srvs.srv import Trigger`;

function NAVIGATE_WAIT(id){ 
	return `\ndef navigate_wait_${id}(x=0, y=0, z=0, speed=0.5, frame_id='body${id}', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry_${id}(frame_id='navigate_target${id}')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;
	}

const LAND_WAIT = () => `\ndef land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(${params.sleep_time})\n`;

const WAIT_YAW = () => `\ndef wait_yaw():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.yaw) < math.radians(${params.yaw_tolerance}):
            return
        rospy.sleep(${params.sleep_time})\n`;

const WAIT_ARRIVAL = () => `\ndef wait_arrival():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;

const ARRIVED = () => `\ndef arrived():
    telem = get_telemetry(frame_id='navigate_target')
    return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}\n`

const GET_DISTANCE = `\ndef get_distance(x, y, z, frame_id):
    telem = get_telemetry(frame_id)
    return math.sqrt((x - telem.x) ** 2 + (y - telem.y) ** 2 + (z - telem.z) ** 2)\n`;

var rosDefinitions = {};
function generateROSDefinitions(id) {
	
	// order for ROS definitions is significant, so generate all ROS definitions as one
	var code = `rospy.init_node('flight')\n\n`;
	if (rosDefinitions.offboard) {
		code += `get_telemetry_${id} = rospy.ServiceProxy('clover${id}/get_telemetry', srv.GetTelemetry)\n`;
		code += `navigate_${id} = rospy.ServiceProxy('clover${id}/navigate', srv.Navigate)\n`;
		if (rosDefinitions.setVelocity) {
			code += `set_velocity_${id} = rospy.ServiceProxy('clover${id}/set_velocity', srv.SetVelocity)\n`;
		}
		if (rosDefinitions.setAttitude) {
			code += `set_attitude_${id} = rospy.ServiceProxy('clover${id}/set_attitude', srv.SetAttitude)\n`;
		}
		if (rosDefinitions.setRates) {
			code += `set_rates_${id} = rospy.ServiceProxy('clover${id}/set_rates', srv.SetRates)\n`;
		}
		code += `land_${id} = rospy.ServiceProxy('clover${id}/land', Trigger)\n`;
	}
	if (rosDefinitions.setEffect) {
		Blockly.Python.definitions_['import_led_effect'] = 'from clover.srv import SetLEDEffect';
		code += `set_effect_${id} = rospy.ServiceProxy('clover${id}/led/set_effect', SetLEDEffect, persistent=True)\n`;
	}
	if (rosDefinitions.setLeds) {
		Blockly.Python.definitions_['import_set_led'] = 'from led_msgs.srv import SetLEDs\nfrom led_msgs.msg import LEDState';
		code += `set_leds_${id} = rospy.ServiceProxy('clover${id}/led/set_leds', SetLEDs, persistent=True)\n`;
	}
	if (rosDefinitions.ledStateArray) {
		Blockly.Python.definitions_['import_led_state_array'] = 'from led_msgs.msg import LEDStateArray';
	}
	if (rosDefinitions.navigateWait) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += NAVIGATE_WAIT(id);
	}
	if (rosDefinitions.landWait) {
		code += LAND_WAIT();
	}
	if (rosDefinitions.waitArrival) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_ARRIVAL();
	}
	if (rosDefinitions.arrived) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += ARRIVED();
	}
	if (rosDefinitions.waitYaw) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_YAW();
	}
	if (rosDefinitions.distance) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += GET_DISTANCE;
	}
	Blockly.Python.definitions_[`ros_${id}`] = code;
}

function initNode(id) {
	Blockly.Python.definitions_['import_rospy'] = 'import rospy';
	generateROSDefinitions(id);
}

function simpleOffboard(id) {
	rosDefinitions.offboard = true;
	Blockly.Python.definitions_['import_srv'] = IMPORT_SRV;
	initNode(id);
}

// Adjust indentation
Blockly.Python.INDENT = '    ';

export function generateUserCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = '_b(%1)\n';
	var code = Blockly.Python.workspaceToCode(workspace);
	return code;
}

function buildFrameId(block) {
	let frame = block.getFieldValue('FRAME_ID').toLowerCase();
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	return `'${frame}${id}'`;
}

Blockly.Python.navigate = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);

	let params = [`x=${x}`, `y=${y}`, `z=${z}`, `frame_id=${frameId}`, `speed=${speed}`];

	simpleOffboard(id);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard(id);

		return `navigate_wait_${id}(${params.join(', ')}),\n`;

	} else {
		if (frameId != 'body') {
			params.push(`yaw=float('nan')`);
		}
		return `navigate_${id}(${params.join(', ')})\n`;
	}
}

Blockly.Python.set_velocity = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	simpleOffboard();

	if (frameId == `'body'`) {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, frame_id=${frameId})\n`;
	} else {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, yaw=float('nan'), frame_id=${frameId})\n`;
	}
}

Blockly.Python.take_off = function(block) {
	simpleOffboard();

	let z = Blockly.Python.valueToCode(block, 'ALT', Blockly.Python.ORDER_NONE);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard();

		return `navigate_wait_${id}(z=${z}, frame_id='body', auto_arm=True)\n`;
	} else {
		return `navigate_{id}(z=${z}, frame_id='body0', auto_arm=True)\n`;
	}
}

Blockly.Python.land = function(block) {
	simpleOffboard();

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.landWait = true;
		simpleOffboard();

		return `land_wait()\n`;
	} else {
		return 'land()\n';
	}
}

Blockly.Python.angle = function(block) {
	// return [block.getFieldValue('ANGLE'), Blockly.Python.ORDER_UNARY_SIGN];
	Blockly.Python.definitions_['import_math'] = 'import math';
	return [`math.radians(${block.getFieldValue('ANGLE')})`, Blockly.Python.ORDER_FUNCTION_CALL];

}

Blockly.Python.set_yaw = function(block) {
	simpleOffboard();
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let code = `navigate(x=float('nan'), y=float('nan'), z=float('nan'), yaw=${yaw}, frame_id=${frameId})\n`;
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.waitYaw = true;
		simpleOffboard();
		code += 'wait_yaw()\n';
	}
	return code;
}

Blockly.Python.wait_arrival = function(block) {
	rosDefinitions.waitArrival = true;
	simpleOffboard();
	return 'wait_arrival()\n';
}

Blockly.Python.get_time = function(block) {
	initNode();
	return ['rospy.get_time()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.arrived = function(block) {
	rosDefinitions.arrived = true;
	simpleOffboard();
	return ['arrived()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.wait = function(block) {
	initNode();
	return `rospy.sleep(${Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_NONE)})\n`;
}

Blockly.Python.setpoint = function(block) {
	var type = block.getFieldValue('TYPE');
	let frameId = buildFrameId(block);
	let vx = Blockly.Python.valueToCode(block, 'VX', Blockly.Python.ORDER_NONE);
	let vy = Blockly.Python.valueToCode(block, 'VY', Blockly.Python.ORDER_NONE);
	let vz = Blockly.Python.valueToCode(block, 'VZ', Blockly.Python.ORDER_NONE);
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let pitch = Blockly.Python.valueToCode(block, 'PITCH', Blockly.Python.ORDER_NONE);
	let roll = Blockly.Python.valueToCode(block, 'ROLL', Blockly.Python.ORDER_NONE);
	let thrust = Blockly.Python.valueToCode(block, 'THRUST', Blockly.Python.ORDER_NONE);

	if (type == 'VELOCITY') {
		rosDefinitions.setVelocity = true;
		simpleOffboard();
		return `set_velocity(vx=${vx}, vy=${vy}, vz=${vz}, frame_id=${frameId}, yaw=float('nan'))\n`;
	} else if (type == 'ATTITUDE') {
		rosDefinitions.setAttitude = true;
		simpleOffboard();
		return `set_attitude(pitch=${pitch}, roll=${roll}, yaw=${yaw}, thrust=${thrust}, frame_id=${frameId})\n`;
	} else if (type == 'RATES') {
		rosDefinitions.setRates = true;
		simpleOffboard();
		return `set_rates(pitch_rate=${pitch}, roll_rate=${roll}, yaw_rate=${yaw}, thrust=${thrust})\n`;
	}
}

Blockly.Python.get_position = function(block) {
	simpleOffboard();
	let frameId = buildFrameId(block);
	var code = `get_telemetry(${frameId}).${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_yaw = function(block) {
	simpleOffboard();
	Blockly.Python.definitions_['import_math'] = 'import math';
	let frameId = buildFrameId(block);
	var code = `math.degrees(get_telemetry(${frameId}).yaw)`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_attitude = function(block) {
	simpleOffboard();
	Blockly.Python.definitions_['import_math'] = 'import math';
	var code = `math.degrees(get_telemetry().${block.getFieldValue('FIELD').toLowerCase()})`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.global_position = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.distance = function(block) {
	rosDefinitions.distance = true;
	simpleOffboard();

	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	return [`get_distance(${x}, ${y}, ${z}, ${frameId})`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.rangefinder_distance = function(block) {
	initNode();
	Blockly.Python.definitions_['import_range'] = 'from sensor_msgs.msg import Range';
	return [`rospy.wait_for_message('rangefinder/range', Range).range`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.mode = function(block) {
	simpleOffboard();
	return [`get_telemetry().mode`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.armed = function(block) {
	simpleOffboard();
	return [`get_telemetry().armed`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.voltage = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('TYPE').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

function parseColor(color) {
	return {
		r: parseInt(color.substr(2, 2), 16),
		g: parseInt(color.substr(4, 2), 16),
		b: parseInt(color.substr(6, 2), 16)
	}
}

const PARSE_COLOR = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}(color):
  return {'r': int(color[1:3], 16), 'g': int(color[3:5], 16), 'b': int(color[5:7], 16)}`;

// TODO: weird code with colour_rgb block
Blockly.Python.set_effect = function(block) {
	rosDefinitions.setEffect = true;
	initNode();

	var effect = block.getFieldValue('EFFECT').toLowerCase();

	if (effect == 'rainbow' || effect == 'rainbow_fill') {
		return `set_effect(effect='${effect}')\n`;
	} else {
		let colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

		if (/^'(.*)'$/.test(colorCode)) { // is simple string
			let color = parseColor(colorCode);
			return `set_effect(effect='${effect}', r=${color.r}, g=${color.g}, b=${color.b})\n`;
		} else {
			let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
			return `set_effect(effect='${effect}', **${parseColor}(${colorCode}))\n`;
		}
	}
}

Blockly.Python.set_led = function(block) {
	rosDefinitions.setLeds = true;
	initNode();

	var index = Blockly.Python.valueToCode(block, 'INDEX', Blockly.Python.ORDER_NONE);
	var colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

	if (/^'(.*)'$/.test(colorCode)) { // is simple string
		let color = parseColor(colorCode);
		return `set_leds([LEDState(index=int(${index}), r=${color.r}, g=${color.g}, b=${color.b})])\n`; // TODO: check for simple int
	} else {
		let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
		return `set_leds([LEDState(index=${index}, **${parseColor}(${colorCode}))])\n`;
	}
}

const GET_LED_COUNT = `led_count = None

def get_led_count():
    global led_count
    if led_count is None:
        led_count = len(rospy.wait_for_message('led/state', LEDStateArray, timeout=10).leds)
    return led_count\n`;

Blockly.Python.led_count = function(block) {
	rosDefinitions.ledStateArray = true;
	initNode();
	Blockly.Python.definitions_['get_led_count'] = GET_LED_COUNT;
	return [`get_led_count()`, Blockly.Python.ORDER_FUNCTION_CALL]
}

function pigpio() {
	Blockly.Python.definitions_['import_pigpio'] = 'import pigpio';
	Blockly.Python.definitions_['init_pigpio'] = 'pi = pigpio.pi()';
}

const GPIO_READ = `\ndef gpio_read(pin):
    pi.set_mode(pin, pigpio.INPUT)
    return pi.read(pin)\n`;

const GPIO_WRITE = `\ndef gpio_write(pin, level):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, level)\n`;

const SET_SERVO = `\ndef set_servo(pin, pwm):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(pin, pwm)\n`;

const SET_DUTY_CYCLE = `\ndef set_duty_cycle(pin, duty_cycle):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_dutycycle(pin, duty_cycle * 255)\n`;

Blockly.Python.gpio_read = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_read'] = GPIO_READ;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	return [`gpio_read(${pin})`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.gpio_write = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_write'] = GPIO_WRITE;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var level = Blockly.Python.valueToCode(block, 'LEVEL', Blockly.Python.ORDER_NONE);
	return `gpio_write(${pin}, ${level})\n`;
}

Blockly.Python.set_servo = function(block) {
	pigpio();
	Blockly.Python.definitions_['set_servo'] = SET_SERVO;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var pwm = Blockly.Python.valueToCode(block, 'PWM', Blockly.Python.ORDER_NONE);
	return `set_servo(${pin}, ${pwm})\n`;
}

Blockly.Python.set_duty_cycle = function(block) {
	pigpio();
	Blockly.Python.definitions_['set_duty_cycle'] = SET_DUTY_CYCLE;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var dutyCycle = Blockly.Python.valueToCode(block, 'DUTY_CYCLE', Blockly.Python.ORDER_NONE);
	return `set_duty_cycle(${pin}, ${dutyCycle})\n`;
}

Blockly.Python.led_height = function(block) {
	simpleOffboard();
	var dropdown_cond2 = block.getFieldValue('COND2');
	var value_cond = Blockly.Python.valueToCode(block, 'COND', Blockly.Python.ORDER_NONE);
	var statements_do = Blockly.Python.statementToCode(block, 'DO');
	return `led_height(${dropdown_cond2}, ${value_cond}, ${statements_do})\n`;
}

//! FORMS (FORMAÇÃO)
// definição
Blockly.Python.forms = function(block) {
	// ativa os nós principais do simplesOffboard (do próprio ROS já)
	simpleOffboard();
	// ativa o navigateWait, bem como ativamos o simpleOffboard acima
	rosDefinitions.navigateWait = true;
	// define os parametros do bloco um a um
	// liga o valor de z e x com os blocos espectivos Z (altura) e X (lado), esses valores erão setados no bloco
	var value_z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	var value_x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	// pega a string dos formatos (SQAURE, TRIANGLE ...)
	let dropdown_forms = block.getFieldValue('Forms');
	// Id do bloco (não fica visível nos blocos)
	let frameId = buildFrameId(block);
	// outro parametro que sera setado no bloco
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);	// define os parametros que serão retornados de acordo com as variaveis setadas aciam	// x e z sao fornecidos pelo bloco, y será nulo para essa translação 	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];	return `navigate_wait(${params.join(', ')}),\n`;	
	// opção para os formatos, dependendo de cada um ele setará os parametros uma certa forma
	// quadrado de 4 lados, com lado X
	if (dropdown_forms == 'SQUARE'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params2 = [`x=${value_x}`, `y=${value_x}`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=0`, `y=${value_x}`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params4 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\nnavigate_wait(${params4.join(', ')}),\n`;	
	}
	// triangulo isóceles com base X 
	if (dropdown_forms == 'TRIANGLE'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params2 = [`x=(${value_x}/2)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\n`;
	}
	// hexagono isóceles regular de base X
	if (dropdown_forms == 'HEXAG'){
	let params = [`x=${value_x}`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params2 = [`x=(5*${value_x}/4)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params3 = [`x=${value_x}`, `y=(${value_x})*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`]
	let params4 = [`x=0`, `y=(${value_x})*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params5 = [`x=(-${value_x}/4)`, `y=(${value_x}/2)*math.sqrt(3)`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	let params6 = [`x=0`, `y=0`, `z=${value_z}`, `frame_id=${frameId}`, `speed=${speed}`];
	return `navigate_wait(${params.join(', ')}),\nnavigate_wait(${params2.join(', ')}),\nnavigate_wait(${params3.join(', ')}),\nnavigate_wait(${params4.join(', ')}),\nnavigate_wait(${params5.join(', ')}),\nnavigate_wait(${params6.join(', ')}),\n`;
	}
	// obs: no return foi usado uma "gambiarra", vários navigate_wait são concatenados com ",\n" entre eles,
	// dessa forma é possível montar uma lista de intruções para serem seguidas
}

