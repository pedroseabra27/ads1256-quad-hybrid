from ads1256_quad_api import create_default_system


def main():
    system = create_default_system()
    system.initialize()
    with system.acquisition_context():
        data = system.read_voltages_numpy(100)
        print("Data shape:", data.shape)
        print("Stats:", system.get_stats())


if __name__ == "__main__":
    main()
