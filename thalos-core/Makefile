MODELS_DIR=src/models

robot:
	@if [ -z "$(NAME)" ]; then \
		echo "Usage: make robot NAME=planar_2r"; \
		exit 1; \
	fi

	@mkdir -p $(MODELS_DIR)/$(NAME)/tests

	@echo "pub mod factory;" > $(MODELS_DIR)/$(NAME)/mod.rs
	@echo "pub mod tests;" >> $(MODELS_DIR)/$(NAME)/mod.rs

	@touch $(MODELS_DIR)/$(NAME)/factory.rs

	@echo "pub mod fk;" > $(MODELS_DIR)/$(NAME)/tests/mod.rs

	@touch $(MODELS_DIR)/$(NAME)/tests/fk.rs

	@echo "Robot model '$(NAME)' created."