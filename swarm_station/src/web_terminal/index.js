// Page js

var terminalCounter = 0;
var nTerminal = 1;

document.getElementById("open-button-terminal").addEventListener("click", function() {
    console.log("Terminal counter: ", terminalCounter)
    if(terminalCounter === 0){
        var nameTerminal = document.createElement("name_"+terminalCounter);
        nameTerminal.className = "nameTerminal";
        nameTerminal.innerHTML = "Terminal " + nTerminal;
        document.getElementById("terminal").appendChild(nameTerminal);

        console.log("Botão 1 apertado");
        terminalCounter++;
        var terminalDiv = document.createElement("div");
        terminalDiv.className = webTerminal;
        document.getElementById("terminal").appendChild(terminalDiv);
        terminalDiv.id = terminalCounter - 1;// adiciona um estilo CSS para ajustar a margem inferior
        // document.body.appendChild(document.createElement("br"));
        // var webTerminal = new webTerminal(document.getElementById("terminal" + terminalCounter));
        console.log("Terminal div: ", terminalDiv);
        let terminal = new webTerminal(terminalDiv.id);
    }
});

document.getElementById("open-terminal").addEventListener("click", function(){
    var nameTerminal = document.createElement("name_"+terminalCounter);
    nameTerminal.className = "nameTerminal";
    nTerminal++;
    nameTerminal.innerHTML = "Terminal " + nTerminal;
    document.getElementById("terminal").appendChild(nameTerminal);

    console.log("Botão 2 apertado");
    console.log("New terminal")
    terminalCounter++;
    var terminalDiv = document.createElement("div");
    terminalDiv.className = webTerminal;
    document.getElementById("terminal").appendChild(terminalDiv);
    terminalDiv.id = terminalCounter - 1;
    terminalDiv.style.marginBottom = "5px"; // adiciona um estilo CSS para ajustar a margem inferior
    // document.body.appendChild(document.createElement("br"));
    // var webTerminal = new webTerminal(document.getElementById("terminal" + terminalCounter));
    console.log("Terminal div: ", terminalDiv);
    let terminal = new webTerminal(terminalDiv.id);
});
