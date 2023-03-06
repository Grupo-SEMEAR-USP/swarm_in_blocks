
import './App.css'
import Front_page from './components/Front_page'
import Header from './components/Header'
import Clover_tools from './components/Clover_tools'
import Easy2Swarm from './components/Easy2Swarm'
import Carrosel from './components/Carrosel'
import Bottom from './components/Bottom'



const App = () => ( 
    <div className="App">
      <Header />
      <Front_page />
      <Easy2Swarm />
      <Clover_tools />
      <Carrosel />
      <Bottom />
    </div>
);


export default App
