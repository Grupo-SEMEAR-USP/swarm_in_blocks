console.log("a")

const arrowFunction = (func) => { //arrow function
    console.log("arrow function was called")
    func()
}

arrowFunction(
    () => {
        console.log("callback")
    }
)

function Construtor() {
    // por padronização deixamos a inicial maiuscula
    this.name = "Name"
    this.data = new Date("01-11-2003")
    this.metodo = () => {
        return "retornando string do metodo"
    }
}

const objeto = new Construtor()

console.log(objeto.name, objeto.data, objeto.metodo())