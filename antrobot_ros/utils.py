import yaml

def load_node_params(yaml_file, node_name):
    """Extract and return parameters for a specific node under /**."""
    with open(yaml_file, 'r') as file:
        full_params = yaml.safe_load(file)
    # Navigate to the parameters under /** and then to the specific node
    node_params = full_params.get('/**', {}).get('ros__parameters', {}).get(node_name, {})

    return node_params
