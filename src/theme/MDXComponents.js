import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import Mermaid from '@theme/Mermaid';

const Highlight = ({children, color}) => (
  <span style={{ backgroundColor: color, borderRadius: '2px', color: '#fff', padding: '0.2rem' }}>
    {children}
  </span>
);

// This wrapper handles the specific <Mermaid chart={`...`} /> syntax
const MermaidWrapper = (props) => {
  // If the user provided a 'chart' prop, use that as the value
  const chartValue = props.chart || props.value || props.children;
  return <Mermaid value={chartValue} {...props} />;
};

export default {
  ...MDXComponents,
  mermaid: MermaidWrapper,
  Mermaid: MermaidWrapper,
  Highlight: Highlight,
};