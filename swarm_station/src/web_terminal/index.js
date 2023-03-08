// Page js

var terminalCounter = 0;
document.getElementById("open-button-terminal").addEventListener("click", function() {
    console.log("Terminal counter: ", terminalCounter)
    if(terminalCounter === 0){
        console.log("Botão 1 apertado");
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
    }
});

document.getElementById("open-terminal").addEventListener("click", function(){
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
