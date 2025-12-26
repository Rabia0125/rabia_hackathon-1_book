---
sidebar_position: 2
slug: cognitive-planning
title: "Cognitive Planning: LLM-Based Task Decomposition"
sidebar_label: "Cognitive Planning"
description: "Master LLM-based task planning to decompose high-level goals into executable robot action sequences with safety validation."
tags:
  - llm
  - gpt-4
  - claude
  - task-planning
  - cognitive-ai
---

# Cognitive Planning: LLM-Based Task Decomposition

## Learning Objectives

After completing this chapter, you will be able to:

1. Explain why LLMs (Large Language Models) are powerful for robotic task planning and how they enable cognitive reasoning
2. Integrate OpenAI GPT-4, Anthropic Claude, or local LLMs (Llama, Mistral) via API or local deployment
3. Design effective prompts that decompose high-level user goals (e.g., "clean the table") into robot action sequences
4. Parse LLM outputs into structured action plans with parameters, preconditions, and postconditions
5. Validate generated plans against robot capabilities (workspace limits, available actions, sensor constraints)
6. Implement dynamic replanning when actions fail during execution (object not found, path blocked, grasp failed)
7. Enforce safety constraints to prevent harmful plans (collision risks, fragile object handling, restricted areas)
8. Maintain conversation context for follow-up commands (e.g., "do that again", "put it on the table")
9. Provide explainability by logging LLM reasoning and decision-making processes
10. Compare cloud-based LLM APIs vs local deployment for robotics applications (latency, cost, privacy)

## Prerequisites

:::info Before You Begin

- **Chapter 1 (Voice-to-Action fundamentals)**: Understanding of speech recognition, intent extraction, and ROS 2 action execution → [Chapter 1: Voice-to-Action](/docs/module-4-vla/voice-to-action)
- **Module 3 Chapter 3 (Nav2 navigation)**: Familiarity with path planning, costmaps, and autonomous navigation → [Module 3 Chapter 3](/docs/module-3-isaac/nav2-humanoids)
- **Python Proficiency**: Comfortable with async/await, API calls, JSON parsing, exception handling
- **API Access**: OpenAI API key (GPT-4) or Anthropic API key (Claude), or local LLM setup (Ollama, LM Studio)
- **Hardware**: NVIDIA GPU (16GB+ VRAM for local LLMs) or cloud API access

:::

---

## 1. Introduction: LLMs for Robotic Planning

In Chapter 1, we enabled voice commands like "move forward 3 meters". But what if a user says **"prepare the table for dinner"**? This requires:

1. Navigate to the cabinet
2. Pick up a plate
3. Navigate to the table
4. Place the plate on the table
5. Return to cabinet
6. Repeat for utensils, cups, etc.

**Single-action voice control** (Chapter 1) can't handle this. We need **cognitive planning**—the ability to decompose high-level goals into executable sequences.

### Why LLMs for Task Planning?

Large Language Models (GPT-4, Claude, Llama) have emergent reasoning capabilities that make them ideal for robotics:

| Capability | Traditional Planning | LLM-Based Planning |
|------------|---------------------|-------------------|
| **Knowledge** | Hardcoded rules | World knowledge (physics, object affordances) |
| **Flexibility** | Brittle (breaks on new scenarios) | Generalizes to novel situations |
| **Natural Language** | Requires formal task specifications | Understands colloquial instructions |
| **Common Sense** | Must explicitly program every rule | Infers implicit constraints (don't put hot pan on plastic) |
| **Ambiguity Handling** | Fails on vague instructions | Asks clarifying questions |

**Example**:

**User**: "Set the table for 4 people"

**Traditional Planner**: ERROR - "set the table" not in action vocabulary

**LLM Planner**:
```
Plan:
1. Navigate to cabinet
2. Grasp plate (repeat 4x)
3. Navigate to table
4. Place plate at position 1-4
5. Repeat for utensils (fork, knife, spoon)
6. Return to origin
```

---

## 2. LLM Integration (API vs Local)

You have two options for deploying LLMs: **cloud APIs** (OpenAI, Anthropic) or **local models** (Llama, Mistral).

### Option 1: OpenAI GPT-4 API

**Pros**: State-of-the-art reasoning, easy setup, no local GPU needed
**Cons**: API costs ($0.01-0.03 per request), network latency (500-2000ms), privacy concerns

```python
# llm_planner_openai.py
import openai
import os
import json

class OpenAIPlanner:
    def __init__(self):
        # Set API key from environment variable
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        self.model = "gpt-4"  # or "gpt-4-turbo" for faster/cheaper

    def generate_plan(self, user_goal: str, robot_capabilities: dict) -> list:
        # Construct prompt with robot capabilities and task description
        prompt = f"""You are a robotic task planner. Given a high-level user goal, generate a detailed action plan.

Available actions:
- navigate(location: str): Move to a named location
- grasp(object: str): Pick up an object
- place(object: str, location: str): Put object at location
- rotate(angle: int): Turn in place by angle degrees
- wait(duration: int): Pause for duration seconds

Robot capabilities:
{json.dumps(robot_capabilities, indent=2)}

User goal: "{user_goal}"

Generate a step-by-step plan as a JSON array of actions:
[
  {{"action": "navigate", "parameters": {{"location": "cabinet"}}}},
  {{"action": "grasp", "parameters": {{"object": "plate"}}}},
  ...
]

Output ONLY the JSON array, no additional text.
"""

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful robotic task planner."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more deterministic planning
                max_tokens=500
            )

            # Parse response
            plan_text = response.choices[0].message.content.strip()

            # Extract JSON (handle markdown code blocks if present)
            if plan_text.startswith("```"):
                plan_text = plan_text.split("```")[1]
                if plan_text.startswith("json"):
                    plan_text = plan_text[4:]

            plan = json.loads(plan_text)
            return plan

        except Exception as e:
            print(f"LLM planning error: {str(e)}")
            return []

# Test the planner
if __name__ == '__main__':
    planner = OpenAIPlanner()

    capabilities = {
        "available_actions": ["navigate", "grasp", "place", "wait"],
        "known_locations": ["kitchen", "table", "cabinet", "bin"],
        "workspace": "indoor office environment"
    }

    goal = "Clean up the toys and put them in the bin"
    plan = planner.generate_plan(goal, capabilities)

    print("Generated Plan:")
    for i, step in enumerate(plan, 1):
        print(f"{i}. {step['action']}({step['parameters']})")
```

**To run**:
```bash
# Set API key
export OPENAI_API_KEY="sk-your-key-here"

# Run planner
python3 llm_planner_openai.py
```

**Expected output**:
```
Generated Plan:
1. navigate({'location': 'toys'})
2. grasp({'object': 'toy'})
3. navigate({'location': 'bin'})
4. place({'object': 'toy', 'location': 'bin'})
5. navigate({'location': 'toys'})
6. grasp({'object': 'toy'})
...
```

### Option 2: Local LLM (Llama 3 via Ollama)

**Pros**: No API costs, no network dependency, full privacy
**Cons**: Requires local GPU (16GB+ VRAM), slower inference (2-5s vs 500ms for GPT-4)

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Download Llama 3 model (7B parameters, 4.7GB)
ollama pull llama3

# Test the model
ollama run llama3 "Generate a plan to clean a room"
```

```python
# llm_planner_local.py
import requests
import json

class LocalLLMPlanner:
    def __init__(self, model="llama3"):
        self.model = model
        self.api_url = "http://localhost:11434/api/generate"  # Ollama API

    def generate_plan(self, user_goal: str, robot_capabilities: dict) -> list:
        prompt = f"""You are a robotic task planner. Generate a JSON action plan for: "{user_goal}"

Available actions: navigate, grasp, place, rotate, wait
Robot capabilities: {json.dumps(robot_capabilities)}

Output ONLY a JSON array of actions."""

        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "temperature": 0.3
        }

        try:
            response = requests.post(self.api_url, json=payload)
            result = response.json()
            plan_text = result['response']

            # Parse JSON from response
            plan = json.loads(plan_text)
            return plan

        except Exception as e:
            print(f"Local LLM error: {str(e)}")
            return []
```

:::tip Cloud vs Local LLMs

| Factor | Cloud API (GPT-4) | Local (Llama 3) |
|--------|-------------------|-----------------|
| **Cost** | $0.01-0.03 per request | Free (after hardware) |
| **Latency** | 500-2000ms | 2000-5000ms |
| **Quality** | Excellent reasoning | Good (improving rapidly) |
| **Privacy** | Data sent to OpenAI | Fully private |
| **Setup** | API key only | GPU, Ollama install |

**Recommended**: Start with GPT-4 API for development, migrate to local LLM if privacy/cost becomes critical.

:::

---

## 3. Task Decomposition with Prompting

The quality of LLM-generated plans depends heavily on **prompt engineering**.

### Effective Prompt Structure

```python
def build_planning_prompt(user_goal: str, robot_state: dict) -> str:
    prompt = f"""You are an expert robotic task planner. Your job is to decompose high-level user goals into executable robot actions.

**Available Actions**:
1. navigate(location: str) - Move to a named location
2. grasp(object: str) - Pick up an object
3. place(object: str, location: str) - Place object at location
4. rotate(angle: int) - Rotate in place (degrees)
5. wait(duration: int) - Pause for duration (seconds)

**Robot Current State**:
- Position: {robot_state['position']}
- Holding: {robot_state['holding_object']}
- Battery: {robot_state['battery_percent']}%
- Known locations: {', '.join(robot_state['known_locations'])}
- Detected objects: {', '.join(robot_state['visible_objects'])}

**Constraints**:
- Robot can only carry ONE object at a time
- Navigation takes ~30 seconds per location
- Grasping requires object to be within 0.5m reach
- Battery below 20% requires charging (navigate to "charging_station")

**User Goal**: "{user_goal}"

**Task**: Generate a valid, efficient plan as a JSON array. Each action must be feasible given current state.

**Output Format**:
```json
[
  {{"action": "navigate", "parameters": {{"location": "cabinet"}}, "reason": "Need to reach objects"}},
  {{"action": "grasp", "parameters": {{"object": "plate"}}, "reason": "Pick up first item"}},
  ...
]
```

Think step-by-step. Ensure the plan is **safe, efficient, and achieves the goal**.
"""
    return prompt
```

**Why This Works**:
1. **Clear action vocabulary**: LLM knows exactly what actions exist
2. **Robot state context**: Prevents invalid plans (e.g., grasping when already holding something)
3. **Explicit constraints**: Enforces physical limitations (battery, reach distance)
4. **Structured output**: JSON format ensures parseable plans
5. **Reasoning field**: Explainability for debugging

---

## 4. Plan Validation

LLMs can hallucinate invalid actions. **Always validate plans before execution**.

```python
# plan_validator.py
class PlanValidator:
    def __init__(self, robot_capabilities: dict):
        self.capabilities = capabilities
        self.valid_actions = ['navigate', 'grasp', 'place', 'rotate', 'wait']

    def validate(self, plan: list) -> tuple[bool, str]:
        """Returns (is_valid, error_message)"""

        for i, step in enumerate(plan):
            # Check action exists
            if step['action'] not in self.valid_actions:
                return False, f"Step {i+1}: Unknown action '{step['action']}'"

            # Check parameters
            if step['action'] == 'navigate':
                location = step['parameters'].get('location')
                if location not in self.capabilities['known_locations']:
                    return False, f"Step {i+1}: Unknown location '{location}'"

            elif step['action'] == 'grasp':
                # Check robot is not already holding something
                if self.robot_state.get('holding_object'):
                    return False, f"Step {i+1}: Cannot grasp, already holding {self.robot_state['holding_object']}"

                obj = step['parameters'].get('object')
                if obj not in self.capabilities['graspable_objects']:
                    return False, f"Step {i+1}: Cannot grasp '{obj}' (not in graspable objects list)"

        return True, "Plan is valid"

# Usage
validator = PlanValidator(robot_capabilities)
is_valid, error_msg = validator.validate(generated_plan)

if not is_valid:
    print(f"Invalid plan: {error_msg}")
    # Ask LLM to regenerate with error feedback
else:
    print("Plan validated, executing...")
    execute_plan(generated_plan)
```

---

## 5. Dynamic Replanning

When actions fail, the robot must adapt.

```python
# dynamic_replanner.py
class DynamicReplanner:
    def __init__(self, llm_planner, validator):
        self.llm = llm_planner
        self.validator = validator
        self.max_replanning_attempts = 3

    def execute_with_replanning(self, plan: list, user_goal: str):
        for i, step in enumerate(plan):
            success = self.execute_action(step)

            if not success:
                # Action failed - trigger replanning
                print(f"Action {i+1} failed: {step['action']}")

                remaining_goal = f"Continue the original goal '{user_goal}' from step {i+1} onwards, considering that {step['action']} failed."

                new_plan = self.llm.generate_plan(remaining_goal, self.get_robot_state())

                # Validate new plan
                is_valid, error = self.validator.validate(new_plan)

                if is_valid:
                    print(f"Replanned successfully: {len(new_plan)} new steps")
                    return self.execute_with_replanning(new_plan, user_goal)
                else:
                    print(f"Replanning failed: {error}")
                    return False

        return True  # All steps succeeded

    def execute_action(self, step: dict) -> bool:
        # Execute action and return success/failure
        # (Integrate with ROS 2 action clients from Chapter 1)
        pass
```

---

## 6. Safety Constraints

Prevent harmful plans with safety rules.

```python
# safety_checker.py
class SafetyChecker:
    def __init__(self):
        self.restricted_areas = ['stairs', 'pool', 'balcony']
        self.fragile_objects = ['glass', 'vase', 'mirror']
        self.max_force = 10.0  # Newtons

    def check_plan(self, plan: list) -> tuple[bool, str]:
        for step in plan:
            if step['action'] == 'navigate':
                loc = step['parameters']['location']
                if loc in self.restricted_areas:
                    return False, f"Safety violation: Cannot navigate to restricted area '{loc}'"

            elif step['action'] == 'grasp':
                obj = step['parameters']['object']
                if obj in self.fragile_objects:
                    # Check force parameter
                    force = step['parameters'].get('force', 5.0)
                    if force > 3.0:
                        return False, f"Safety violation: Force {force}N too high for fragile object '{obj}'"

        return True, "Plan passes safety checks"
```

---

## 7. Conversation Context

Handle follow-up commands by maintaining dialogue history.

```python
# conversation_manager.py
class ConversationManager:
    def __init__(self, llm_planner):
        self.llm = llm_planner
        self.history = []  # List of (user_command, executed_plan) tuples

    def process_command(self, user_command: str):
        # Check for pronoun references ("do that again", "move it there")
        if self.is_reference(user_command):
            resolved_command = self.resolve_reference(user_command)
            user_command = resolved_command

        # Generate plan
        plan = self.llm.generate_plan(user_command, self.get_robot_state())

        # Save to history
        self.history.append((user_command, plan))

        return plan

    def is_reference(self, command: str) -> bool:
        pronouns = ['that', 'it', 'there', 'again', 'same']
        return any(pronoun in command.lower() for pronoun in pronouns)

    def resolve_reference(self, command: str) -> str:
        if not self.history:
            return command  # No history to reference

        last_command, last_plan = self.history[-1]

        # Simple resolution: "do that again" → repeat last command
        if 'again' in command.lower() or 'repeat' in command.lower():
            return last_command

        # TODO: More sophisticated reference resolution with LLM
        return command
```

---

## Summary and Next Steps

Congratulations! You've learned how to integrate LLM-based cognitive planning into humanoid robots.

### What You Accomplished

- ✅ Integrated LLMs (GPT-4, Claude, Llama) for natural language task planning
- ✅ Designed effective prompts for decomposing high-level goals into action sequences
- ✅ Validated LLM-generated plans against robot capabilities and constraints
- ✅ Implemented dynamic replanning for failure recovery
- ✅ Enforced safety constraints to prevent harmful actions
- ✅ Maintained conversation context for follow-up commands

### Key Takeaways

1. **LLMs Enable Cognitive Reasoning**: Transforms robots from command executors to intelligent assistants
2. **Prompt Engineering is Critical**: Well-designed prompts with constraints and examples yield better plans
3. **Always Validate Plans**: LLMs can hallucinate invalid actions—validation prevents execution errors
4. **Safety is Non-Negotiable**: Safety checkers must run before executing any LLM-generated plan

### What's Next?

In **Chapter 3: Capstone Project**, you'll integrate everything:

- Combine voice control (Chapter 1) with cognitive planning (Chapter 2)
- Integrate Isaac ROS perception (Module 3) for object detection
- Deploy the complete VLA system in Isaac Sim for end-to-end testing
- Execute benchmark tasks with multimodal feedback (voice + visual)
- Deploy to physical humanoid hardware

[Begin Chapter 3: Capstone Project →](/docs/module-4-vla/capstone-project)

---

**Chapter Navigation**:

- ← Previous: [Chapter 1: Voice-to-Action](/docs/module-4-vla/voice-to-action)
- → Next: [Chapter 3: Capstone Project](/docs/module-4-vla/capstone-project)
- ↑ Back to Module 4: [Module 4 Index](/docs/module-4-vla/)

---

*Chapter 2 of Module 4: The AI-Robot Brain (VLA)*
