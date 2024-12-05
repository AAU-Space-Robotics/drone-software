import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sympy import symbols, Matrix, sympify
import xml.etree.ElementTree as ET
import re

class LinearizeNode(Node):
    def __init__(self):
        super().__init__('linearize_node')
        self.get_logger().info("Linearize Node Started")

        # Load robot models from XML file
        self.models = self.load_robot_models_from_xml('/home/daroe/drone-software/src/sym_tf_lib/resource/model.xml')

        # With the robot models loaded, the linerazation can begin. The model is defined as tau = M(q) * q'' + V(q, q') + G(q). This will be transformed into a linear system.
        # The linear system is defined as x' = A * x + B * u. First it needs to be put in the fomr x' = f(x, u), and then it can be linearized around working points.
        # The linearization is done by computing the Jacobians of the system dynamics f(x, u) with respect to the state x and the input u.

        #print self.models['mass_matrix']['matrix']
        
        self.linearize()

    def get_matrix(self, models, matrix_type):
        for model in models:
            if model[matrix_type]:
                MassMatrix = model[matrix_type]["matrix"]
                return MassMatrix

    def get_model_parameter(self, models, matrix_type):
        for model in models:
            if model[matrix_type]:
                param = model[matrix_type]["parameters"]
                return param
            
    def linearize(self):
        MassMatrix = self.get_matrix(self.models, "mass_matrix")
        VelocityTerms = self.get_matrix(self.models, "velocity_terms")
        GravityVector = self.get_matrix(self.models, "gravity_vector")
        tau = self.get_matrix(self.models, "control_inputs")

        print(MassMatrix)
        print(VelocityTerms)
        print(GravityVector)
        print(tau)

        if MassMatrix and VelocityTerms and GravityVector and tau:
            acceleration = sympify(MassMatrix.inv() * (tau - VelocityTerms - GravityVector))
        else:
            self.get_logger().error("One or more matrices/parameters are missing.")


        #Continue from here


       

    def load_robot_models_from_xml(self, xml_file):
        """
        Parse the XML file and extract robot models, equations, and parameters.
        """
        def extract_symbols(expression):
            """Extract all unique symbols from a given expression string."""
            reserved_names = {'cos', 'sin', 'tan', 'sqrt', 'pi', 'exp', 'log'}
            symbol_names = re.findall(r'[a-zA-Z_][a-zA-Z0-9_]*', expression)
            filtered_names = [name for name in symbol_names if name not in reserved_names]
            return symbols(set(filtered_names))

        def parse_matrix(element):
            """Parse a matrix or vector element and return a dictionary with matrix data and parameters."""
            # Extract entries
            entries = element.find('entries')
            if entries is None:
                raise ValueError("Matrix element missing 'entries' tag.")
            
            # Extract dimensions from attributes
            rows_attr = element.get('rows')
            cols_attr = element.get('cols')
            if not rows_attr or not cols_attr:
                raise ValueError(f"Matrix size must be specified in 'rows' and 'cols' attributes for {element.tag}.")
            
            try:
                rows = int(rows_attr)
                cols = int(cols_attr)
            except ValueError:
                raise ValueError(f"Invalid dimensions in matrix: rows={rows_attr}, cols={cols_attr}")

            # Collect all entry tags (assuming each is a scalar expression)
            entry_tags = [entry for entry in entries]
            if len(entry_tags) != rows * cols:
                raise ValueError(f"Number of matrix entries ({len(entry_tags)}) does not match specified size ({rows}x{cols}).")

            # Collect all symbols in the matrix entries
            all_symbols = set()
            for entry in entry_tags:
                all_symbols.update(extract_symbols(entry.text))

            # Declare symbols explicitly
            symbol_dict = {symbol.name: symbol for symbol in all_symbols}

            # Parse each entry expression
            entry_expressions = []
            for entry in entry_tags:
                try:
                    expr = sympify(entry.text, locals=symbol_dict)
                    entry_expressions.append(expr)
                except Exception as e:
                    raise ValueError(f"Error parsing matrix entry '{entry.text}': {e}")

            # Reshape entries into the defined structure
            reshaped_entries = [
                entry_expressions[i * cols:(i + 1) * cols]
                for i in range(rows)
            ]

            matrix_data = {
                "matrix": Matrix(reshaped_entries),
                "parameters": {}
            }

            # Extract parameters
            parameters_element = element.find('parameters')
            if parameters_element is not None:
                for param in parameters_element:
                    param_name = param.tag
                    param_values = param.text.split()
                    matrix_data["parameters"][param_name] = symbols(param_values)

            return matrix_data

        tree = ET.parse(xml_file)
        root = tree.getroot()

        models = []
        for model in root.findall('model'):
            model_data = {
                'name': model.get('name'),
                'mass_matrix': None,
                'velocity_terms': None,
                'gravity_vector': None,
                'control_inputs': []
            }

            # Parse mass matrix
            mass_matrix = model.find('mass_matrix')
            if mass_matrix:
                model_data['mass_matrix'] = parse_matrix(mass_matrix)

            # Parse velocity terms
            velocity_terms = model.find('velocity_terms')
            if velocity_terms:
                model_data['velocity_terms'] = parse_matrix(velocity_terms)

            # Parse gravity vector
            gravity_vector = model.find('gravity_vector')
            if gravity_vector:
                model_data['gravity_vector'] = parse_matrix(gravity_vector)

            # Parse control inputs
            control_inputs = model.find('control_inputs')
            if control_inputs:
                model_data['control_inputs'] = parse_matrix(control_inputs)

            models.append(model_data)

        return models

def main(args=None):
    rclpy.init(args=args)
    node = LinearizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
