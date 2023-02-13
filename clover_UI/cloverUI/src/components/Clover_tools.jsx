import './styles/Clover_tools.css'


function Clover_tools() {

    return (
      <div id="Clover_tools">
        <div class='container'>
            <div id='c_green_gradient'></div>
            <div id='sub_text'>
                Clover kit
                <p />
                Tools
                <div class='text3'>
                    Set of tools for using Clover, in this section these features will be displayed on the screen simultaneously for handling the drone and these applications.</div>
                <a href='ground_station/ground_station.html'>
                    <button class='button'> 
                        Get Started
                    </button>
                </a>
            </div>
            <div id='tools_cards'>
                <div id='c'>
                    <img src='/assets/img/3D_view.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>3D view</div>
                        3d visualization of clover and surrounding environment.
                    </div>
                </div>

                <div id='c_excep'>
                    <img src='/assets/img/Clover_documentation.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>Clover documentation</div>
                        Access to the source documentation of the drone developed by the company Coex.                    
                    </div>
                </div>

                <div id='c'>
                    <img src='/assets/img/Clover_blocks.svg' id=''/>
                    <div id='text_card'>
                        <div id='text1_card'>Clover blocks</div>
                        Code development through blocks to control a clover.                    </div>
                </div>
            </div>
        </div>
      </div>
    )
}

export default Clover_tools