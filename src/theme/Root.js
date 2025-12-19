import React, { useState } from 'react';

export default function Root({children}) {
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState([]);
  // ADD THIS LINE: To track if chat is open or closed
  const [isOpen, setIsOpen] = useState(false);

  const handleSend = async () => {
    if (!input) return;
    const newMessages = [...messages, { text: input, user: true }];
    setMessages(newMessages);
    setInput('');

    try {
      const response = await fetch('https://Bariqa-Humanoid-Robotics.hf.space/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: input }),
      });
      const data = await response.json();
      setMessages([...newMessages, { text: data.response, user: false }]);
    } catch (error) {
      setMessages([...newMessages, { text: "Error connecting to AI.", user: false }]);
    }
  };

  return (
    <>
      {children}
      
      {/* THE BLUE DOT BUTTON */}
      <div 
        className="chat-toggle-button" 
        onClick={() => setIsOpen(!isOpen)}
        style={{
          position: 'fixed', bottom: '20px', right: '20px',
          width: '60px', height: '60px', borderRadius: '50%',
          backgroundColor: '#007bff', color: 'white',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          cursor: 'pointer', zIndex: 2000, fontSize: '24px',
          boxShadow: '0 4px 12px rgba(0,0,0,0.2)'
        }}
      >
        {isOpen ? '✕' : '💬'}
      </div>

      {/* THE CHAT WINDOW (Only shows if isOpen is true) */}
      {isOpen && (
        <div className="chatbot-container" style={{
          position: 'fixed', bottom: '90px', right: '20px',
          width: '350px', backgroundColor: 'white', borderRadius: '10px',
          boxShadow: '0 5px 20px rgba(0,0,0,0.2)', zIndex: 2000,
          display: 'flex', flexDirection: 'column', overflow: 'hidden'
        }}>
          <div className="chat-header" style={{ padding: '15px', background: '#007bff', color: 'white', fontWeight: 'bold' }}>
            Robotics AI Assistant
          </div>
          <div className="chat-messages" style={{ height: '300px', overflowY: 'auto', padding: '10px' }}>
            {messages.map((m, i) => (
              <div key={i} style={{ textAlign: m.user ? 'right' : 'left', margin: '5px' }}>
                <small style={{ color: '#888' }}>{m.user ? 'You' : 'AI'}:</small>
                <div style={{ 
                  background: m.user ? '#007bff' : '#f0f0f0', 
                  color: m.user ? 'white' : 'black',
                  padding: '8px 12px', borderRadius: '10px', display: 'inline-block'
                }}>
                  {m.text}
                </div>
              </div>
            ))}
          </div>
          <div className="chat-input-area" style={{ padding: '10px', borderTop: '1px solid #eee', display: 'flex' }}>
            <input 
              style={{ flex: 1, padding: '8px', border: '1px solid #ddd', borderRadius: '4px' }}
              value={input} 
              onChange={(e) => setInput(e.target.value)} 
              placeholder="Ask about humanoids..." 
            />
            <button onClick={handleSend} style={{ marginLeft: '5px', padding: '8px 15px', background: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}>
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
}