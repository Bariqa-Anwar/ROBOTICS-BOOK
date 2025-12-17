import React from 'react';
import Layout from '@theme/Layout';

export default function Home() {
  return (
    <Layout
      title={`Welcome to the Book`}
      description="Physical AI & Humanoid Robotics: ROS 2 and NVIDIA Isaac Sim Capstone Guide">
      <main>
        <div style={{ padding: '20px', textAlign: 'center' }}>
          <h1>Physical AI & Humanoid Robotics</h1>
          <p>
            This site hosts the complete documentation for the Spec-Driven Book project. 
            Use the **Modules** link in the navigation bar to start reading the content.
          </p>
          <a href="/docs/introduction/docusaurus-setup" style={{
              fontSize: '1.2em',
              padding: '10px 20px',
              backgroundColor: '#303846',
              color: 'white',
              borderRadius: '5px',
              textDecoration: 'none',
          }}>
            Start Reading Now
          </a>
        </div>
      </main>
    </Layout>
  );
}