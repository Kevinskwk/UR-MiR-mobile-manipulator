import openai
import base64

# Load your OpenAI API key
openai.api_key = "YOUR API KEY"

# Function to encode the image into base64
def encode_image(image_path):
    """Encode image to base64 format."""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


# Define the task planning prompt with text and base64 encoded image input
def generate_task_plan(task_description, image_path=None):
    # Prepare the image as a base64 encoded string
    base64_image = encode_image(image_path) if image_path else None

    # Create the content with text and base64 image
    content_message = [
        {
            "type": "text",
            "text": task_description
        }
    ]

    if base64_image:
        content_message.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"
            }
        })

    # Create the prompt
    messages = [
        {"role": "system", "content": """You are highly skilled in robotic task planning, breaking down intricate and long-term tasks into distinct primitive actions, and describe as "find points relevant to the task". Don't say too much irrelevant thing.
        Each action need to clearly specificed to be either a navigation (moving in the room), or a grasp, or a place, and need to be decribe as find points relevant to the task. For example: Nagivation Task, find points to navigate close to the corner desk, Or Grasp, FInd points to grasp sth.
        """},
        {"role": "user", "content": content_message}
    ]

    # Call the OpenAI API to generate a task plan
    response = openai.chat.completions.create(
        model="gpt-4-turbo",  # Adjust the model as necessary
        messages=messages,
        max_tokens=400,
        temperature=0.7,
        n=1
    )

    # Extract and return the plan from the response
    return response.choices[0].message.content.strip()


# Example task description and image path (user can change this)
task_description = "Grab the can on the corner desk and then put it on the square table in the middle of the sofa."
image_path = "office.jpg"  # Replace with the actual image path

# Call the function to generate the task plan
task_plan = generate_task_plan(task_description, image_path)
print(f"Generated Task Plan:\n{task_plan}")