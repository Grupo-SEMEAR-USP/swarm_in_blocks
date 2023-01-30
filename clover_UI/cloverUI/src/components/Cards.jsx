import './styles/Carrosel.css'

function Cards(props) {
    return (
      <div id="Cards">
        <img src={props.img_src} id='card_img'/>
        <div id='card_title'>{props.title}</div>
        <div id='card_lin'></div>
        <div id='card_subtitle'>{props.subtitle}</div>
      </div>
    )
}

export default Cards