import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App'
import './index.css'


export const sizes = {
  sm: '640px',
  md: '768px',
  lg: '1024px',
  xl: '1280px',
  '2xl': '1536px',
};

function isMatch(media) {
  const query = `(min-width: ${sizes[media]})`;
  return window.matchMedia(query).matches;
}

function findClosest(queries) {
  for (let i = queries.length - 1; i >= 0; i--) {
      if (isMatch(queries[i])) {
          return queries[i];
      }
  }
  return 'sm';
}


export const useClosestMedia = () => {
  const [closest, setClosest] = useState('sm');

  useEffect(() => {
      const listener = () => setClosest(findClosest(queries));
      listener();
      window.addEventListener('resize', listener);
      return () => window.removeEventListener('resize', listener); //Cleanup
  }, []);

  return closest;
};



ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
)
