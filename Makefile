.PHONY: production-gate test

production-gate:
	python3 scripts/production_gate.py

test:
	python3 -m unittest discover -s tests -p 'test_*.py'
