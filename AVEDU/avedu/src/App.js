import { Routes, Route, Navigate } from 'react-router-dom';
import Home from './pages/Home';
import Learn from './pages/Learn';
import UnitPage from './pages/UnitPage';
import LearnLevel from './pages/LearnLevel';
import IDETestPage from './pages/IDETestPage';

export default function App() {
  return (
    <Routes>
      <Route path="/" element={<Home />} />

      <Route path="/learn" element={<Learn />}>
        <Route index element={<div className="placeholder">Select a Unit</div>} />
        <Route path=":unitSlug" element={<UnitPage />}>
          <Route index element={<div className="placeholder">Choose a level</div>} />
          <Route path=":levelSlug" element={<LearnLevel />} />
        </Route>
      </Route>

      <Route path="/research" element={<IDETestPage />} />

      <Route path="*" element={<Navigate to="/learn" replace />} />
    </Routes>
  );
}
