import { useEffect, useRef, useState } from "react";
import "./App.css";

const FIELD_WIDTH = 16.54;
const FIELD_HEIGHT = 8.21;

function createWaypoint(x, y, rotation = 16, entryAngle = null, velocity = 0) 
{return {x, y, rotation, entryAngle, velocity};}

function fieldToPercent(x, y) {
  return {
    left: `${(x / FIELD_WIDTH) * 100}%`,
    top: `${100 - (y / FIELD_HEIGHT) * 100}%`
  };
}

function createPathPoint(waypoint) {
  return `${waypoint.x},${FIELD_HEIGHT - waypoint.y}`;
}

function App() {
  const fieldRef = useRef(null);

  const [waypoints, setWaypoints] = useState([
    createWaypoint(2.5, 5.8, 16),
    createWaypoint(5.0, 5.8, 16),
    createWaypoint(7.5, 5.2, 16),
    createWaypoint(10.0, 4.2, 16),
    createWaypoint(12.5, 3.5, 16),
  ]);

  const [selectedIndex, setSelectedIndex] = useState(null);
  const [draggingIndex, setDraggingIndex] = useState(null);

  const [dragOffset, setDragOffset] = useState({x: 0, y: 0});

  const [rotatingIndex, setRotatingIndex] = useState(null);

  const rotationStartRef = useRef({pointerAngle: 0, waypointRotation: 0});

  /* ========================================================= */
  /* KEYBOARD CONTROLS */
  /* ========================================================= */

  useEffect(() => {
    function handleKeyboard(event) {
      const tagName = event.target.tagName;

      if (
        tagName === "INPUT" ||
        tagName === "TEXTAREA" ||
        tagName === "SELECT"
      ) {return;}

      /* ENTER = ADD WAYPOINT */

      if (event.key === "Enter") {
        event.preventDefault();
        addWaypoint();
        return;
      }

      /* BACKSPACE = DELETE WAYPOINT */

      if (event.key === "Backspace") {
        event.preventDefault();

        if (
          selectedIndex === null ||
          selectedIndex === 0 ||
          selectedIndex === waypoints.length - 1
        ) {return;}

        const deletedIndex = selectedIndex;

        setWaypoints((current) =>
          current.filter((_, index) => index !== deletedIndex)
        );

        setSelectedIndex(
          (current) => {
            if (current === null) {return null;}
            const newLength = waypoints.length - 1;
            if (current >= newLength) {return newLength - 1;}
            return current;
          }
        );
        return;
      }

      /* NOTHING TO NAVIGATE */

      if (selectedIndex === null ||waypoints.length === 0)
         {return;}

      /* LEFT / UP = PREVIOUS */

      if (event.key === "ArrowLeft" ||event.key === "ArrowUp") {
        event.preventDefault();

        setSelectedIndex(
          (selectedIndex - 1 + waypoints.length) % waypoints.length
        );
        return;
      }

      /* RIGHT / DOWN = NEXT */

      if (event.key === "ArrowRight" ||event.key === "ArrowDown"){
        event.preventDefault();

        setSelectedIndex(
          (selectedIndex + 1) % waypoints.length
        );
      }
    }

    window.addEventListener("keydown", handleKeyboard);

    return () => {window.removeEventListener("keydown",handleKeyboard);};
  }, [selectedIndex, waypoints.length]);

  /* ========================================================= */
  /* MOUSE POSITION → FIELD POSITION */
  /* ========================================================= */

  function mouseToField(event) {
    const field = fieldRef.current;

    if (!field) {
      return null;
    }

    const rect = field.getBoundingClientRect();

    return {
      x: Math.max(0, Math.min(FIELD_WIDTH, (event.clientX - rect.left) / rect.width * FIELD_WIDTH)),
      y: Math.max(0,Math.min(FIELD_HEIGHT, (1 - (event.clientY - rect.top) / rect.height) * FIELD_HEIGHT)),
    };
  }

  /* ========================================================= */
  /* ADD WAYPOINT */
  /* ========================================================= */

  function addWaypoint() {
    if (waypoints.length === 0) {
      setWaypoints([createWaypoint(3, 4, 16)]);

      setSelectedIndex(0);
      return;
    }

    const insertIndex =
      selectedIndex === null
        ? waypoints.length
        : selectedIndex + 1;

    const previous = waypoints[insertIndex - 1];

    const next = waypoints[insertIndex];

    let newX = 3;
    let newY = 4;
    let newRotation = 16;

    if (previous && next) {
      
      newX = (previous.x + next.x) / 2;
      newY = (previous.y + next.y) / 2;
      newRotation = (previous.rotation + next.rotation) / 2;
    } else if (previous) {
      
      newX = Math.min(previous.x + 1, FIELD_WIDTH - 0.5);
      newY = previous.y;
      newRotation = previous.rotation;
    }

    const newWaypoint = createWaypoint(newX, newY, newRotation);

    setWaypoints((current) => {
      const updated = [...current];
      updated.splice(insertIndex, 0, newWaypoint);
      return updated;
    });

    setSelectedIndex(insertIndex);
  }

  /* ========================================================= */
  /* UPDATE WAYPOINT */
  /* ========================================================= */

  function updateWaypoint(index, property, value) {
    setWaypoints((currentWaypoints) => {
      return currentWaypoints.map((waypoint, waypointIndex) => {
        if (waypointIndex !== index) {
          return waypoint;
        }

        return {
          ...waypoint,
          [property]: value,
        };
      });
    });
  }

  /* ========================================================= */
  /* UPDATE NUMBER */
  /* ========================================================= */

  function updateNumberProperty(index, property, value) {
    const parsed = Number(value);

    updateWaypoint(
      index,
      property,
      Number.isFinite(parsed)
        ? parsed
        : 0
    );
  }

  /* ========================================================= */
  /* UPDATE OPTIONAL NUMBER */
  /* ========================================================= */

  function updateOptionalNumber(index, property,value) {
    if (value === "") {
      updateWaypoint(index, property, null);
      return;
    }

    const parsed = Number(value);

    updateWaypoint(
      index,
      property,
      Number.isFinite(parsed)
        ? parsed
        : null
    );
  }

  /* ========================================================= */
  /* START MOVING WAYPOINT */
  /* ========================================================= */

  function startDragging(event, index) {
    if (event.target.closest(".rotation-handle")) {
      return;
    }

    event.preventDefault();
    event.stopPropagation();

    const position = mouseToField(event);

    if (!position) {
      return;
    }

    const waypoint = waypoints[index];

    setDragOffset({
      x: position.x - waypoint.x,
      y: position.y - waypoint.y,
    });

    setSelectedIndex(index);
    setDraggingIndex(index);

    try {
      event.currentTarget.setPointerCapture(event.pointerId);
    } catch {
      // Pointer capture isn't required.
    }
  }

  /* ========================================================= */
  /* MOVE WAYPOINT */
  /* ========================================================= */

  function handleFieldPointerMove(event) {
    if (draggingIndex === null) {
      return;
    }

    const position = mouseToField(event);

    if (!position) {
      return;
    }

    const newX = position.x - dragOffset.x;

    const newY = position.y - dragOffset.y;

    setWaypoints((current) =>
      current.map((waypoint, index) => {
        if (index !== draggingIndex) {
          return waypoint;
        }

        return {
          ...waypoint,
          x: Math.max(0, Math.min(FIELD_WIDTH, newX)),
          y: Math.max(0, Math.min(FIELD_HEIGHT, newY)),
        };

      })
    );
  }

  /* ========================================================= */
  /* STOP MOVING */
  /* ========================================================= */

  function stopDragging() {
    setDraggingIndex(null);
  }

  /* ========================================================= */
  /* START ROTATION */
  /* ========================================================= */

  function startRotation(event, index) {
    event.preventDefault();
    event.stopPropagation();
    const button = event.currentTarget.closest(".waypoint");

    if (!button) {
      return;
    }

    const rect = button.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    const pointerAngle =
      Math.atan2(
        event.clientY - centerY,
        event.clientX - centerX
      ) * (180 / Math.PI);

    rotationStartRef.current = {
      pointerAngle,
      waypointRotation: waypoints[index].rotation || 0,
    };

    setSelectedIndex(index);
    setRotatingIndex(index);

    try {
      event.currentTarget.setPointerCapture(event.pointerId);
    } catch {
      // Pointer capture isn't required.
    }
  }

  /* ========================================================= */
  /* ROTATION */
  /* ========================================================= */

  useEffect(() => {
    if (rotatingIndex === null) {
      return;
    }

    function handleRotationMove(event) {
      const waypointButton =document.querySelector(`.waypoint[data-index="${rotatingIndex}"]`);

      if (!waypointButton) {
        return;
      }

      const rect = waypointButton.getBoundingClientRect();

      const centerX = rect.left + rect.width / 2;

      const centerY = rect.top + rect.height / 2;

      const currentPointerAngle =
        Math.atan2(
          event.clientY - centerY,
          event.clientX - centerX
        ) * (180 / Math.PI);

      let delta = currentPointerAngle - rotationStartRef.current.pointerAngle;

      if (delta > 180) {
        delta -= 360;
      }

      if (delta < -180) {
        delta += 360;
      }

      const newRotation = rotationStartRef.current.waypointRotation + delta;

      setWaypoints((current) =>
        current.map((waypoint, index) =>
          index === rotatingIndex
            ? {
                ...waypoint,
                rotation: newRotation,
              }
            : waypoint
        )
      );
    }

    function handleRotationEnd() {
      setRotatingIndex(null);
    }

    window.addEventListener("pointermove", handleRotationMove);

    window.addEventListener("pointerup", handleRotationEnd);

    window.addEventListener("pointercancel", handleRotationEnd);

    return () => {
      window.removeEventListener("pointermove", handleRotationMove);
      window.removeEventListener("pointerup", handleRotationEnd);
      window.removeEventListener("pointercancel", handleRotationEnd);
    };
  }, [rotatingIndex]);

  /* ========================================================= */
  /* FIELD CLICK */
  /* ========================================================= */

  function handleFieldClick(event) {
    if (event.target !== event.currentTarget) {return;}
    setSelectedIndex(null);
  }

  /* ========================================================= */
  /* DELETE WAYPOINT */
  /* ========================================================= */

  function deleteWaypoint(index) {
    const isStart = index === 0;
    const isEnd = index === waypoints.length - 1;
    if (isStart || isEnd) {return;}

    setWaypoints((current) =>
      current.filter((_, waypointIndex) =>waypointIndex !== index)
    );

    setSelectedIndex((current) => {
      if (current === null) {return null;}
      if (current === index) {
        return Math.min(index, waypoints.length - 2);
      }
      if (current > index) {return current - 1;}

      return current;
    });
  }

  /* ========================================================= */
  /* WAYPOINT TYPE */
  /* ========================================================= */

  function getWaypointType(index) {
    if (index === 0) {return "Start";}
    if (index === waypoints.length - 1) {return "End";}
    return `Waypoint ${index}`;
  }

  /* ========================================================= */
  /* WAYPOINT CLICK */
  /* ========================================================= */

  function handleWaypointClick(event, index) {
    event.stopPropagation();
    setSelectedIndex(index);
  }

  function handleWaypointHeaderClick(index) {
    setSelectedIndex((current) =>
      current === index
        ? null
        : index
    );
  }

  /* ========================================================= */
  /* RENDER WAYPOINT */
  /* ========================================================= */

  function renderWaypoint(waypoint, index) {
    const position = fieldToPercent(waypoint.x,waypoint.y);
    const isSelected = selectedIndex === index;
    const isStart = index === 0;
    const isEnd = index === waypoints.length - 1;

    const pointColor = isStart
      ? "green"
      : isEnd
      ? "red"
      : "orange";

    return (
      <button
        key={index}
        data-index={index}
        type="button"
        className={`
          waypoint
          ${isSelected ? "selected" : ""}
          ${isStart ? "start" : ""}
          ${isEnd ? "end" : ""}
          ${
            rotatingIndex === index
              ? "rotating"
              : ""
          }
        `}
        style={position}
        onPointerDown={(event) =>startDragging(event, index) }
        onClick={(event) =>handleWaypointClick(event,index)}
      >

        <span
          className={`waypoint-number ${pointColor}`}
          style={{transform: `rotate(${waypoint.rotation || 0}deg)`}}
        >

          <span className="center-dot" />

          <span
            className="rotation-handle"
            onPointerDown={(event) =>startRotation(event,index)}
          />

        </span>

      </button>
    );
  }

  /* ========================================================= */
  /* WAYPOINT CARD */
  /* ========================================================= */

  function renderWaypointCard(waypoint, index) {
    const isSelected = selectedIndex === index;
    const isStart = index === 0;
    const isEnd = index === waypoints.length - 1;
    const pointColor = isStart
      ? "green"
      : isEnd
      ? "red"
      : "orange";

    return (

      <div
        key={index}
        className={`
          waypoint-card
          ${isSelected ? "expanded" : ""}
          ${pointColor}
        `}
      >

        <button
          type="button"
          className="waypoint-card-header"
          onClick={() =>handleWaypointHeaderClick(index)}
        >

          <span className={`sidebar-dot ${pointColor}`}/>

          <span className="waypoint-card-name">
            
            <span className="waypoint-card-title"> {getWaypointType(index)} </span>

            <span className="waypoint-card-position">
              {waypoint.x.toFixed(2)}
              {" , "}
              {waypoint.y.toFixed(2)}
            </span>

          </span>

          <span
            className={`waypoint-chevron ${isSelected ? "open" : ""}`}
            aria-hidden="true"
          >
            ›
          </span>
          
        </button>

        {isSelected && (
          <div className="waypoint-properties">

            {/* REFERENCE */}

            <div className="property-section">

              <div className="property-section-title">Reference</div>

              <div className="property-grid">

                <label className="property-field">
                  
                  <span>X</span>

                  <input
                    type="number"
                    step="0.01"
                    value={waypoint.x}
                    onChange={(event) =>updateNumberProperty(index, "x", event.target.value)}
                  />

                </label>

                <label className="property-field">
                  
                  <span>Y</span>

                  <input
                    type="number"
                    step="0.01"
                    value={waypoint.y}
                    onChange={(event) =>updateNumberProperty(index, "y", event.target.value)}
                  />

                </label>

              </div>

            </div>

            {/* ROTATION */}

            <div className="property-section">

              <div className="property-section-title">Rotation</div>

              <label className="property-field full">

                <span>Rotation °</span>

                <input
                  type="number"
                  step="0.1"
                  value={Number(waypoint.rotation || 0).toFixed(1)}
                  onChange={(event) =>updateNumberProperty(index, "rotation", event.target.value)}
                />

              </label>

            </div>

            {/* ENTRY ANGLE */}

            <div className="property-section">

              <div className="property-section-title">Entry Angle</div>

              <label className="property-field full">

                <span>Angle °</span>

                <input
                  type="number"
                  step="0.1"
                  placeholder="None"
                  value={waypoint.entryAngle ?? ""}
                  onChange={(event) =>updateOptionalNumber(index, "entryAngle", event.target.value)}
                />

              </label>

              <div className="property-help">Optional target entry angle</div>

            </div>

            {/* VELOCITY */}

            <div className="property-section">

              <div className="property-section-title">End Velocity</div>

              <label className="property-field full">
                
                <span>Velocity</span>

                <input
                  type="number"
                  step="0.1"
                  min="0"
                  value={waypoint.velocity}
                  onChange={(event) =>updateNumberProperty(index,"velocity", event.target.value)}
                />
                
              </label>

            </div>

            {/* DELETE */}

            {!isStart && !isEnd && (
              <button
                type="button"
                className="delete-waypoint-button"
                onClick={() =>deleteWaypoint(index)}
              >
                Delete Waypoint
              </button>
            )}

            {(isStart || isEnd) && (
              <div className="protected-note">
                {isStart
                  ? "Start waypoint cannot be deleted."
                  : "End waypoint cannot be deleted."}
              </div>
            )}

          </div>
        )}

      </div>

    );

  }

  /* ========================================================= */
  /* RENDER */
  /* ========================================================= */

  return (
    <div className="app">

      {/* TOP BAR */}

      <header className="topbar">

        <div className="brand">

          <div className="brand-mark">S</div>

          <div>

            <div className="brand-title">Syrup</div>

            <div className="brand-subtitle">FRC Autonomous Editor</div>
          
          </div>
        
        </div>

        <div className="topbar-title">Autonomous Path</div>

        <button className="export-button" type="button">Export</button>
      
      </header>

      {/* MAIN */}

      <main className="workspace">
        <section className="editor">

          {/* TOOLBAR */}

          <div className="toolbar">
            
            <div>
              
              <div className="toolbar-title">Field</div>

              <div className="toolbar-subtitle">Drag waypoints to create your path</div>
            
            </div>

            <div className="toolbar-actions">
              
              <button
                className="add-button"
                type="button"
                onClick={addWaypoint}
              >
                + Add Waypoint
              
              </button>
            
            </div>
          
          </div>

          {/* EDITOR AREA */}

          <div className="editor-body">

            {/* FIELD */}

            <div
              className="field-container"
              onPointerMove={handleFieldPointerMove}
              onPointerUp={stopDragging}
              onPointerCancel={stopDragging}
              onPointerLeave={stopDragging}
            >

              <div
                ref={fieldRef}
                className="field"
                onClick={handleFieldClick}
              >

                {/* GRID */}

                <div className="grid">
                  {Array.from({length: 11,}).map((_, index) => (
                    <div
                      key={`vertical-${index}`}
                      className="grid-vertical"
                      style={{
                        left: `${(index / 10) *100}%`
                      }}
                    />
                  ))}

                  {Array.from({length: 7,}).map((_, index) => (
                    <div
                      key={`horizontal-${index}`}
                      className="grid-horizontal"
                      style={{top: `${(index / 6) *100}%`}}
                    />
                  ))}

                </div>

                {/* CENTER LINE */}

                <div className="center-line" />

                {/* FIELD LABELS */}

                <div className="field-label blue">BLUE</div>
                <div className="field-label red">RED</div>

                {/* PATH */}

                <svg
                  className="path"
                  viewBox={`0 0 ${FIELD_WIDTH} ${FIELD_HEIGHT}`}
                  preserveAspectRatio="none"
                >

                  <polyline
                    points={waypoints
                      .map(createPathPoint)
                      .join(" ")}
                    fill="none"
                    stroke="#f47b32"
                    strokeWidth="0.07"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />

                </svg>

                {/* WAYPOINTS */}

                {waypoints.map(renderWaypoint)}
              
              </div>

            </div>

            {/* WAYPOINT SIDEBAR */}

            <aside className="waypoint-panel">

              <div className="waypoint-panel-header">

                <div>

                  <div className="panel-title">Waypoints</div>

                  <div className="panel-subtitle">{waypoints.length} points in path</div>
                
                </div>
              
              </div>

              <div className="waypoint-list">{waypoints.map(renderWaypointCard)}</div>
            
            </aside>
          
          </div>

          {/* BOTTOM BAR */}

          <div className="bottom-bar">

            <div className="path-info">

              <span className="info-label">Waypoints</span>

              <span className="info-value">{waypoints.length}</span>
           
            </div>

            <div className="path-info">

              <span className="info-label">Selected</span>

              <span className="info-value">
                {selectedIndex === null
                  ? "None"
                  : getWaypointType(selectedIndex)}
              </span>

            </div>

            <div className="hint">
              Drag the square to move • drag the
              border dot to rotate
            </div>

          </div>

        </section>

      </main>

    </div>
    
  );
}

export default App;