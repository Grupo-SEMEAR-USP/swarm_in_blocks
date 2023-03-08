// const { FitAddon } = require("xterm-addon-fit");

// Create a conection between the frontend and backend 
const socket = new WebSocket("ws://localhost:6060");

// Setting control variables
var getUser = true;
var userData;
var firstMsg = true;
var firstConfigs = true;
var welcomeMsg;
var currentDirectory = [];
var auxCurDir=false;
var aux1 = 0;
var lastclientInput;
var newLine = false;
var numberLine = 0;
var tabCount = 0;
var lastTabInput;


let fitAddon = new FitAddon.FitAddon();

// Instantiate a xterm object
var terminal = new Terminal({
    cursorBlink: true,
    'theme': { foreground: "#44ff9b", background: '#000000'},
    'allowTransparency': true,
    scrollback: 1000,
});

terminal.loadAddon(fitAddon);

if (terminal != null) {
    terminal.open(document.getElementById('terminal'));
    // Fit terminal element in the definied container
    fitAddon.fit();
  } else {
    console.log("Failed to create terminal object");
  }

function init() {
    
    if (terminal._initialized) return;

    terminal._initialized = true;

    terminal.prompt = () => {
        terminal.write(`\r\n${userData.username}@${userData.hostname}:$ `);
    };
    
    prompt(terminal);

    terminal.onData(dataInput => {

        if (firstConfigs===true){
            // clientInput = String.fromCharCode(99) + String.fromCharCode(100) + String.fromCharCode(32) + String.fromCharCode(46) + String.fromCharCode(46);
            clientInput = '';
            sendToServer(terminal, clientInput)
        }
        // Check if the client is writing reached a new line
        if(terminal._core.buffer.x === 1){
            newLine = true;
            numberLine += 1;
        }

        switch (dataInput) {

            case '\u001B[A': // Up Arrow
                clearInput(clientInput)
                clientInput = dataInput;
                socket.send(clientInput);
                clientInput = '';
                lastclientInput = dataInput;
                break;

            case '\u0003': // Ctrl+C
                terminal.write('^C');
                prompt(terminal);
                lastclientInput = dataInput;
                break;

            case '\r': // Enter
                sendToServer(terminal, clientInput);
                // clearInput(clientInput);
                // socket.send(clientInput + '\n');
                clientInput = '';
                lastclientInput = dataInput;
                newLine = false;
                numberLine = 0;
                break;
                
            case '\u007F': // Delete
                lastclientInput = dataInput;
                // Call function to get the current file directory the client is at
                let curDirLenght = getCurrentDir()
                var userLenght = userData.username.length + userData.hostname.length + curDirLenght + 4;
                // Condition to not delete further than the username + hostname + current directory
                if (terminal._core.buffer.x > userLenght) {
                    terminal.write('\b \b');
                    if (clientInput.length > 0) {
                        clientInput = clientInput.substr(0, clientInput.length - 1);
                    }
                }else if(terminal._core.buffer.x === 0){ // Client will move one line up after deleting the input
                    terminal.write('\x1b[A'); // move pra linha anterior
                    terminal.write('\x1b[999C'); // move para o final da linha
                    terminal.write('\x1b[0K'); // apaga o último dígito
                    numberLine = numberLine - 2;
                }else if(newLine===true && numberLine > 0){ // Client is not at the initial line, so it has no limits when deleting inputs
                    terminal.write('\b \b');
                    if (clientInput.length > 0) {
                        clientInput = clientInput.substr(0, clientInput.length - 1);
                    }
                }
                break;

            case '\u0009': // Tab
            // Autocomplete clientInput
                clientInput += dataInput;
                sendToServer(terminal, clientInput, tab = true);
                clientInput = '';
                lastclientInput = dataInput;
                break;
            
            case '\u001B[C': // Right arrow
            // allows text navigation
                terminal.write('\u001B[C');
                // clearInput(clientInput)
                socket.send(clientInput);
                clientInput = '';
                lastclientInput = dataInput;
                break;

            case '\u001B[D': // Left arrow
            // allows text navigation
                terminal.write('\u001B[D');
                // clearInput(clientInput)
                // clientInput = dataInput;
                socket.send(clientInput);
                clientInput = '';
                lastclientInput = dataInput;
                break;               

            default:
                if(dataInput >= String.fromCharCode(0x20) && dataInput <= String.fromCharCode(0x7E) || dataInput >= '\u00a0'){
                    clientInput += dataInput;
                    lastclientInput = dataInput;
                    // Do not run the clientInput, only writes it on the terminal
                    terminal.write(dataInput);
                }
                break; 
        }
    });
}

// Mudar função para ter possibilidade de deletar n letras ou deletar tudo
function clearInput(clientInput) {
    console.log("apagou")
    var inputLengh = clientInput.length;
    for (var i = 0; i < inputLengh; i++) {
        terminal.write('\b \b');
    }
}

function prompt(terminal) {
    if(firstMsg === true){
        // terminal.write('\r\n░██████╗███████╗███╗░░░███╗███████╗░█████╗░██████╗░');
        // terminal.write('\r\n██╔════╝██╔════╝████╗░████║██╔════╝██╔══██╗██╔══██╗');
        // terminal.write('\r\n╚█████╗░█████╗░░██╔████╔██║█████╗░░███████║██████╔╝');
        // terminal.write('\r\n░╚═══██╗██╔══╝░░██║╚██╔╝██║██╔══╝░░██╔══██║██╔══██╗');
        // terminal.write('\r\n██████╔╝███████╗██║░╚═╝░██║███████╗██║░░██║██║░░██║');
        // terminal.write('\r\n╚═════╝░╚══════╝╚═╝░░░░░╚═╝╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝\r\n');
        terminal.write('\r\n╔═══╗╔╗╔╗╔╗╔═══╗╔═══╗╔═╗╔═╗    ╔══╗╔═╗ ╔╗    ╔══╗ ╔╗   ╔═══╗╔═══╗╔╗╔═╗╔═══╗');
        terminal.write('\r\n║╔═╗║║║║║║║║╔═╗║║╔═╗║║║╚╝║║    ╚╣╠╝║║╚╗║║    ║╔╗║ ║║   ║╔═╗║║╔═╗║║║║╔╝║╔═╗║');
        terminal.write('\r\n║╚══╗║║║║║║║║ ║║║╚═╝║║╔╗╔╗║     ║║ ║╔╗╚╝║    ║╚╝╚╗║║   ║║ ║║║║ ╚╝║╚╝╝ ║╚══╗');
        terminal.write('\r\n╚══╗║║╚╝╚╝║║╚═╝║║╔╗╔╝║║║║║║     ║║ ║║╚╗║║    ║╔═╗║║║ ╔╗║║ ║║║║ ╔╗║╔╗║ ╚══╗║');
        terminal.write('\r\n║╚═╝║╚╗╔╗╔╝║╔═╗║║║║╚╗║║║║║║    ╔╣╠╗║║ ║║║    ║╚═╝║║╚═╝║║╚═╝║║╚═╝║║║║╚╗║╚═╝║');
        terminal.write('\r\n╚═══╝ ╚╝╚╝ ╚╝ ╚╝╚╝╚═╝╚╝╚╝╚╝    ╚══╝╚╝ ╚═╝    ╚═══╝╚═══╝╚═══╝╚═══╝╚╝╚═╝╚═══╝');
        terminal.write('\r\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
        terminal.write('\r\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n');
        firstMsg = false;
        welcomeMsg = 'Welcome to the web terminal application, press enter to start using';
        terminal.write(``) ;
        terminal.write(`\r\n ${welcomeMsg} `);      
        clientInput = '';

//           ╔╗       ╔╗                       ╔╗ 
//           ║║      ╔╝╚╗                      ║║ 
// ╔╗╔╗╔╗╔══╗║╚═╗    ╚╗╔╝╔══╗╔═╗╔╗╔╗╔╗╔═╗ ╔══╗ ║║ 
// ║╚╝╚╝║║╔╗║║╔╗║     ║║ ║╔╗║║╔╝║╚╝║╠╣║╔╗╗╚ ╗║ ║║ 
// ╚╗╔╗╔╝║║═╣║╚╝║     ║╚╗║║═╣║║ ║║║║║║║║║║║╚╝╚╗║╚╗
// ╚╝╚╝ ╚══╝╚══╝     ╚═╝╚══╝╚╝ ╚╩╩╝╚╝╚╝╚╝╚═══╝╚═╝
                                        
    }else{
        clientInput = '';
        terminal.write(``) ;    
        terminal.write(`\r\n${userData.username}@${userData.hostname}:$ `);
    }
}

socket.onmessage = (event) => {
    let aux2;
    if(getUser===true){
        // Receive user data
        userData = JSON.parse(event.data);
        getUser = false;
    }else if(firstConfigs===true){
            //dont write anything to the client
            firstConfigs = false;
             return;
    }else if(auxCurDir===true){
            var currentDirectoryBack = event.data;
            if (currentDirectoryBack[0]==='/'){
                currentDirectory = currentDirectoryBack.split("\n")[0];
            }
            aux1 += 1;
            if(aux1 === 2){
                auxCurDir = false;
                aux1 = 0;
                aux2 = true;
            }
    }else if(lastclientInput === '\u0009'){
        if(tabCount === 0){
            // ignore the first time it runs
            tabCount +=1
        }else{
            terminal.write(event.data);
            tabCount = 0;
        }
        // console.log("Rodou o treco do tab")
        // console.log(event.data)
        // terminal.write(event.data[event.data.lenght - 1]) 

    }else if(lastclientInput != '\u007F'){
        terminal.write(event.data);         
    }
}


function sendToServer(terminal, clientInput, tab=false) {
    if(firstConfigs===true){
        clearInput(welcomeMsg + ' ');
        socket.send(clientInput + '\n');
        return;
    }
    else if(tab===true){
        socket.send(clientInput);
        return;
    }else{
        // Enter clientInput
        clearInput(clientInput);
        socket.send(clientInput + '\n');
        return;
    }
            
}

function getCurrentDir(){
    // Get current directory
    auxCurDir = true;
    let pwdclientInput = String.fromCharCode(112) + String.fromCharCode(119) + String.fromCharCode(100);
    socket.send(pwdclientInput + '\n');
    // console.log(`CurrentDir: ${currentDirectory}`);
    let curDirLenght = currentDirectory.length - userData.username.length - 6;
    // console.log(`CurrentDirLenght: ${curDirLenght}`);
    return curDirLenght;
}

init();

